//! Isolate which piece of `hyg24` delivers the gain (`MAX_CAND=24` vs adjacent-start
//! suppression vs plateau-center tie-break) + standalone 1,000-minute noise-only FAR.
//!
//! Usage:
//!   cargo run --release --example proto_hyg24_ablation -- [trials_per_snr] [noise_mins]

use std::thread;
use urad950::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use urad950::ldpc::{self, BpScratch, N_BITS};
use urad950::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS, S0, S1, S2, SUBCHANNELS_HZ};
use urad950::rx::{FFT_M, FFT_N, FIRST_BIN, N_BINS, SYM_SAMPLES};
use urad950::rx60::FOLD_SYMS;
use urad950::{
    encode_tones, Callsign, Decoded, Frame, Modulator, DEFAULT_SPACING_HZ, MAX_DECODED,
};

const RATE: u32 = 24_000;
const SIGMA: f64 = 1_500.0;
const REPEATS: usize = 5;
const BLOCK: usize = 192;
const RAMP_SAMPLES_6K: usize = 60;
const HALF_BIN_Q4: usize = 8;
const TONE_SPACING_Q4: usize = 68;
const COMPACT_CHANNELS: usize = SUBCHANNELS_HZ.len();
const COMPACT_FIRST_SYM: usize = 1 + data_sym_pos(0);
const COMPACT_LAST_SYM: usize = FOLD_SYMS - urad950::phy::N_SYMBOLS + data_sym_pos(N_DATA_SYMS - 1);
const COMPACT_SYMS: usize = COMPACT_LAST_SYM - COMPACT_FIRST_SYM + 1;

static LP_24K_TO_6K: [i16; 31] = lowpass::<31>(2_550.0 / 24_000.0);

static NOMINAL_HALF_BINS: [usize; COMPACT_CHANNELS] = {
    let mut bins = [0; COMPACT_CHANNELS];
    let mut ch = 0;
    while ch < COMPACT_CHANNELS {
        bins[ch] = (SUBCHANNELS_HZ[ch] as usize * (FFT_N * 2) + 3_000) / 6_000 - FIRST_BIN * 2;
        ch += 1;
    }
    bins
};

#[derive(Clone, Copy)]
struct Cfg {
    max_cand: usize,
    adj_sym_sup: bool,
    plateau_center: bool,
}

struct Rx512 {
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: usize,
    decim_count: u32,
    fold: [[u8; N_BINS]; FOLD_SYMS],
    compact: [[[u8; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
}

impl Rx512 {
    fn new() -> Self {
        Self {
            decim: Decimator::new(),
            pcm_ring: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            fold: [[0; N_BINS]; FOLD_SYMS],
            compact: [[[0; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
        }
    }
    fn reset(&mut self) {
        self.decim.reset();
        self.pcm_ring.fill(0);
        self.pcm_pos = 0;
        self.decim_count = 0;
        for r in &mut self.fold { r.fill(0); }
        for c in &mut self.compact { for s in c { s.fill(0); } }
    }
    fn start_subslot(&mut self) {
        self.decim.reset();
        self.pcm_ring.fill(0);
        self.pcm_pos = 0;
        self.decim_count = 0;
    }
    fn push_samples(&mut self, samples: &[i16]) {
        for &s in samples {
            if let Some(y) = self.decim.push(s, &LP_24K_TO_6K, 4) {
                self.pcm_ring[self.pcm_pos] = y;
                self.pcm_pos += 1;
                if self.pcm_pos == SYM_SAMPLES { self.pcm_pos = 0; }
                self.decim_count = self.decim_count.wrapping_add(1);
                if self.decim_count.is_multiple_of(SYM_SAMPLES as u32) {
                    let sym = (self.decim_count / SYM_SAMPLES as u32) as usize - 1;
                    if sym < FOLD_SYMS { self.on_symbol(sym); }
                }
            }
        }
    }
    #[inline]
    fn quantize(lp: i32, floor: i32, span: i32) -> i32 {
        ((lp - floor + 3).max(0) * 6 / span).clamp(0, 6)
    }
    fn on_symbol(&mut self, sym: usize) {
        let mut z = [0i16; FFT_N];
        let mut idx = (self.pcm_pos + RAMP_SAMPLES_6K) % SYM_SAMPLES;
        for sample in z.iter_mut().take(SYM_SAMPLES - RAMP_SAMPLES_6K) {
            *sample = self.pcm_ring[idx];
            idx = if idx + 1 == SYM_SAMPLES { 0 } else { idx + 1 };
        }
        let shift = cfft(&mut z, FFT_M);
        let mut cur_bins = [0u8; N_BINS];
        let mut out_i = 0usize;
        let mut sum = 0u32;
        let mut peak = 0u8;
        let mut hist = [0u8; 32];
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            let v = lp.clamp(0, 255) as u8;
            cur_bins[out_i] = v;
            sum += v as u32;
            if v > peak { peak = v; }
            hist[usize::from(v >> 3)] += 1;
            out_i += 1;
        });
        let floor = (sum / N_BINS as u32) as i32;
        let peak_span = (i32::from(peak) - floor).clamp(18, 96);
        for b in 0..N_BINS {
            let q = Self::quantize(i32::from(cur_bins[b]), floor, peak_span) as u8;
            self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
        }
        if sym >= COMPACT_FIRST_SYM && sym <= COMPACT_LAST_SYM {
            let mut seen = 0usize;
            let mut upper = 0i32;
            for (bucket, &count) in hist.iter().enumerate().rev() {
                seen += usize::from(count);
                if seen > N_BINS / 20 { upper = (bucket * 8 + 7) as i32; break; }
            }
            let local_span = (upper - floor).clamp(18, 96);
            if local_span < peak_span {
                let c_sym = sym - COMPACT_FIRST_SYM;
                for ch in 0..COMPACT_CHANNELS {
                    for tone in 0..4usize {
                        let pos = NOMINAL_HALF_BINS[ch] * 8 + tone * TONE_SPACING_Q4;
                        let bin = pos >> 4;
                        let frac = (pos & 15) as i32;
                        let old_q4 = Self::quantize(i32::from(cur_bins[bin]), floor, peak_span) * (16 - frac)
                            + Self::quantize(i32::from(cur_bins[bin + 1]), floor, peak_span) * frac;
                        let loc_q4 = Self::quantize(i32::from(cur_bins[bin]), floor, local_span) * (16 - frac)
                            + Self::quantize(i32::from(cur_bins[bin + 1]), floor, local_span) * frac;
                        let added = ((loc_q4 - old_q4 + 8) >> 4).max(0) as u8;
                        if added > 0 {
                            self.compact[ch][c_sym][tone] = self.compact[ch][c_sym][tone].saturating_add(added).min(7);
                        }
                    }
                }
            }
        }
    }
    #[inline]
    fn sample_tone_q4(&self, sym: usize, f0_half: usize, tone: u8) -> i32 {
        let pos_x16 = f0_half * HALF_BIN_Q4 + usize::from(tone) * TONE_SPACING_Q4;
        let b = pos_x16 >> 4;
        let frac = (pos_x16 & 15) as i32;
        if b + 1 >= N_BINS || sym >= FOLD_SYMS { return 0; }
        i32::from(self.fold[sym][b]) * (16 - frac) + i32::from(self.fold[sym][b + 1]) * frac
    }
    fn decode_primary(&self, start_sym: u8, f0_half: u16, scratch: &mut BpScratch) -> Option<Frame> {
        let mut inter_llr = [0i32; N_BITS];
        for d in 0..N_DATA_SYMS {
            let s = start_sym as usize + data_sym_pos(d);
            let m0 = self.sample_tone_q4(s, f0_half as usize, 0);
            let m1 = self.sample_tone_q4(s, f0_half as usize, 1);
            let m2 = self.sample_tone_q4(s, f0_half as usize, 2);
            let m3 = self.sample_tone_q4(s, f0_half as usize, 3);
            inter_llr[2 * d] = m0.max(m1) - m2.max(m3);
            inter_llr[2 * d + 1] = m0.max(m3) - m1.max(m2);
        }
        let mut deinter = [0i32; N_BITS];
        let mut max_abs = 1i32;
        for i in 0..N_BITS {
            let v = inter_llr[interleave_idx(i)];
            deinter[i] = v;
            max_abs = max_abs.max(v.abs());
        }
        let mut llr_i8 = [0i8; N_BITS];
        for i in 0..N_BITS {
            llr_i8[i] = ((deinter[i] * 60) / max_abs).clamp(-120, 120) as i8;
        }
        ldpc::decode_bp(&llr_i8, scratch, 35).and_then(|bits| Frame::from_bits78(&bits))
    }
    fn decode_compact(&self, ch: usize, start_sym: u8, scratch: &mut BpScratch) -> Option<Frame> {
        let f0_half = NOMINAL_HALF_BINS[ch];
        let mut interleaved = [0i8; N_BITS];
        for d in 0..N_DATA_SYMS {
            let s = usize::from(start_sym) + data_sym_pos(d);
            let c_sym = s - COMPACT_FIRST_SYM;
            let tones: [i32; 4] = core::array::from_fn(|t| {
                self.sample_tone_q4(s, f0_half, t as u8) + i32::from(self.compact[ch][c_sym][t]) * 16
            });
            interleaved[2 * d] = (tones[0].max(tones[1]) - tones[2].max(tones[3])).clamp(-127, 127) as i8;
            interleaved[2 * d + 1] = (tones[0].max(tones[3]) - tones[1].max(tones[2])).clamp(-127, 127) as i8;
        }
        let mut deinterleaved = [0i8; N_BITS];
        let mut scale = 1i8;
        for i in 0..N_BITS {
            let v = interleaved[interleave_idx(i)];
            deinterleaved[i] = v;
            scale = scale.max(v.abs());
        }
        for v in &mut deinterleaved {
            *v = (i16::from(*v) * 60 / i16::from(scale)) as i8;
        }
        ldpc::decode_bp(&deinterleaved, scratch, 35).and_then(|bits| Frame::from_bits78(&bits))
    }
    fn finish_with(&self, cfg: Cfg, out: &mut [Decoded; MAX_DECODED]) -> usize {
        let mut best: Vec<(i32, u8, u16)> = vec![(i32::MIN, 0, 0); cfg.max_cand];
        let mut n_best = 0usize;
        let max_f0_half = ((N_BINS - 1) * 16 - 1 - 3 * TONE_SPACING_Q4) / HALF_BIN_Q4;
        let mut compact_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];

        for start_sym in 1usize..=(FOLD_SYMS - 110) {
            let mut row_scores = [0i32; 320];
            let mut nom_ch = 0usize;
            for f0_half in 0..=max_f0_half {
                let mut score = 0i32;
                for (k, &t) in S0.iter().enumerate() {
                    let s = start_sym + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0) + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2) + self.sample_tone_q4(s, f0_half, 3)) >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S1.iter().enumerate() {
                    let s = start_sym + 18 + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0) + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2) + self.sample_tone_q4(s, f0_half, 3)) >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S2.iter().enumerate() {
                    let s = start_sym + 104 + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0) + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2) + self.sample_tone_q4(s, f0_half, 3)) >> 2;
                    score += sig - bg;
                }
                row_scores[f0_half] = score;
                if nom_ch < COMPACT_CHANNELS && f0_half == NOMINAL_HALF_BINS[nom_ch] {
                    let bs = &mut compact_sync[nom_ch];
                    if score > bs[0].0 { bs[1] = bs[0]; bs[0] = (score, start_sym as u8); }
                    else if score > bs[1].0 { bs[1] = (score, start_sym as u8); }
                    nom_ch += 1;
                }
            }
            let mut f0_half = 0usize;
            while f0_half <= max_f0_half {
                let score = row_scores[f0_half];
                let mut picked = f0_half;
                if cfg.plateau_center {
                    let mut end = f0_half;
                    while end + 1 <= max_f0_half && row_scores[end + 1] == score { end += 1; }
                    picked = (f0_half + end) / 2;
                    f0_half = end + 1;
                } else {
                    f0_half += 1;
                }
                if n_best == cfg.max_cand && score <= best[cfg.max_cand - 1].0 { continue; }
                let mut dup_idx = None;
                for (i, item) in best.iter().enumerate().take(n_best) {
                    let sm = if cfg.adj_sym_sup { item.1.abs_diff(start_sym as u8) <= 1 } else { item.1 == start_sym as u8 };
                    let df = if cfg.adj_sym_sup { 4 } else { 3 };
                    if sm && (item.2 as usize).abs_diff(picked) <= df { dup_idx = Some(i); break; }
                }
                if let Some(di) = dup_idx {
                    if score > best[di].0 { best[di] = (score, start_sym as u8, picked as u16); }
                } else if n_best < cfg.max_cand {
                    best[n_best] = (score, start_sym as u8, picked as u16);
                    n_best += 1;
                } else {
                    best[cfg.max_cand - 1] = (score, start_sym as u8, picked as u16);
                }
                for i in 1..n_best {
                    let mut j = i;
                    while j > 0 && best[j].0 > best[j - 1].0 { best.swap(j, j - 1); j -= 1; }
                }
            }
        }
        let mut scratch = BpScratch::new();
        let mut n_out = 0usize;
        for &(score, start_sym, f0_half) in best.iter().take(n_best) {
            if n_out >= MAX_DECODED { break; }
            if let Some(frame) = self.decode_primary(start_sym, f0_half, &mut scratch) {
                if out[..n_out].iter().any(|d| d.frame == frame) { continue; }
                out[n_out] = Decoded {
                    frame,
                    audio_hz: (((FIRST_BIN as u32 * 2 + f0_half as u32) * 6000) / 1024) as u16,
                    time_offset_ms: (start_sym as i16 - 5) * 80,
                    snr_db: (-22 + score / 180).clamp(-24, 10) as i8,
                };
                n_out += 1;
            }
        }
        for rank in 0..2 {
            for (ch, starts) in compact_sync.iter().enumerate() {
                if n_out == MAX_DECODED { break; }
                let (score, start_sym) = starts[rank];
                let to = (start_sym as i16 - 5) * 80;
                if out[..n_out].iter().any(|d| d.audio_hz.abs_diff(SUBCHANNELS_HZ[ch]) <= 18 && d.time_offset_ms == to) {
                    continue;
                }
                if let Some(frame) = self.decode_compact(ch, start_sym, &mut scratch) {
                    if !out[..n_out].iter().any(|d| d.frame == frame) {
                        out[n_out] = Decoded { frame, audio_hz: SUBCHANNELS_HZ[ch], time_offset_ms: to, snr_db: (-22 + score / 180).clamp(-24, 10) as i8 };
                        n_out += 1;
                    }
                }
            }
        }
        n_out
    }
}

fn cq(call: &[u8], grid: u16) -> Frame {
    Frame::Cq { sender: Callsign::encode(call).unwrap(), grid4: grid, tx_pwr: 10, reserved: 0 }
}
fn synth(frame: Frame, hz: u32, db: f64, oms: usize) -> Vec<i16> {
    let np = SIGMA.powi(2) * 2500.0 / (RATE / 2) as f64;
    let amp = (2.0 * np * 10f64.powf(db / 10.0)).sqrt().round().max(1.0) as i16;
    let mut w = vec![0i16; RATE as usize * 10];
    let mut tx = Modulator::new(encode_tones(frame), RATE, hz, DEFAULT_SPACING_HZ, amp);
    for s in &mut w[oms * 24..] { if let Some(v) = tx.next_sample() { *s = v; } else { break; } }
    w
}
fn noise(seed: u64) -> Vec<i16> {
    let mut st = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ 0xA5A5_5A5A_1234_5678;
    if st == 0 { st = 1; }
    (0..RATE as usize * 10 * REPEATS).map(|_| {
        let mut s = -6.0;
        for _ in 0..12 { st ^= st << 13; st ^= st >> 7; st ^= st << 17; s += (st >> 16) as u32 as f64 / u32::MAX as f64; }
        (s * SIGMA).round().clamp(-32_000.0, 32_000.0) as i16
    }).collect()
}
fn fit50(snrs: &[f64], hits: &[usize], n: usize) -> f64 {
    let (mut a, mut b) = (22.0, 1.0);
    for _ in 0..40 {
        let (mut g0, mut g1, mut h00, mut h01, mut h11) = (0.0, 0.0, 1e-6, 0.0, 1e-6);
        for (&x, &k) in snrs.iter().zip(hits) {
            let p = 1.0 / (1.0 + (-(a + b * x).clamp(-30.0, 30.0)).exp());
            let w = n as f64 * p * (1.0 - p);
            g0 += k as f64 - n as f64 * p; g1 += (k as f64 - n as f64 * p) * x;
            h00 += w; h01 += w * x; h11 += w * x * x;
        }
        let det = h00 * h11 - h01 * h01;
        if det.abs() < 1e-12 { break; }
        a += (h11 * g0 - h01 * g1) / det;
        b += (-h01 * g0 + h00 * g1) / det;
    }
    -a / b
}

fn main() {
    let trials: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(200);
    let n_mins: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(1000);
    let cfgs = [
        ("base",         Cfg { max_cand: 10, adj_sym_sup: false, plateau_center: false }),
        ("cand24_only",  Cfg { max_cand: 24, adj_sym_sup: false, plateau_center: false }),
        ("sup_only",     Cfg { max_cand: 10, adj_sym_sup: true,  plateau_center: false }),
        ("plateau_only", Cfg { max_cand: 10, adj_sym_sup: false, plateau_center: true  }),
        ("hyg24_full",   Cfg { max_cand: 24, adj_sym_sup: true,  plateau_center: true  }),
    ];
    let target = cq(b"NQ6N", 1234);
    let nbr = cq(b"WP2Z", 2345);
    let snrs = [-22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];
    let nt = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    println!("=== HYG24 ABLATION ({trials} trials/SNR, single shared FFT/fold per minute) ===");
    for (ci, &(name, hz, has_nbr)) in [
        ("nbr18_2175_+5Hz", 2180u32, true),
        ("nbr18_2175_-10Hz", 2165u32, true),
        ("nbr18_2175_0Hz", 2175u32, true),
    ].iter().enumerate() {
        let mut hits = vec![[0usize; 5]; snrs.len()];
        let mut tot_b = [0usize; 5];
        let mut tot_c = [0usize; 5];
        for (si, &db) in snrs.iter().enumerate() {
            let tw = synth(target, hz, db, 400);
            let nw = has_nbr.then(|| synth(nbr, 1950, db + 18.0, 440));
            let chunk = trials.div_ceil(nt);
            thread::scope(|s| {
                let mut hs = Vec::new();
                for ti in 0..nt {
                    let st = ti * chunk; let en = (st + chunk).min(trials);
                    if st >= en { continue; }
                    let (twr, nwr) = (&tw, &nw);
                    hs.push(s.spawn(move || {
                        let mut rx = Rx512::new();
                        let mut lh = [0usize; 5];
                        let mut lb = [0usize; 5];
                        let mut lc = [0usize; 5];
                        for i in st..en {
                            let nz = noise(3_000_000 + (ci as u64)*100_000 + (si as u64)*10_000 + i as u64);
                            rx.reset();
                            for sub in 0..REPEATS {
                                if sub > 0 { rx.start_subslot(); }
                                let base = sub * twr.len();
                                for blk in 0..(twr.len() / BLOCK) {
                                    let off = blk * BLOCK;
                                    let samp: [i16; BLOCK] = core::array::from_fn(|k| {
                                        let mut a = i32::from(nz[base + off + k]) + i32::from(twr[off + k]);
                                        if let Some(n) = nwr { a += i32::from(n[off + k]); }
                                        a.clamp(-32_768, 32_767) as i16
                                    });
                                    rx.push_samples(&samp);
                                }
                            }
                            let mut ok = [false; 5];
                            for (k, &(_, cfg)) in cfgs.iter().enumerate() {
                                let mut out = [Decoded::EMPTY; MAX_DECODED];
                                let n = rx.finish_with(cfg, &mut out);
                                ok[k] = out[..n].iter().any(|d| d.frame == target);
                                if ok[k] { lh[k] += 1; }
                            }
                            for k in 1..5 {
                                if ok[0] && !ok[k] { lb[k] += 1; }
                                if !ok[0] && ok[k] { lc[k] += 1; }
                            }
                        }
                        (lh, lb, lc)
                    }));
                }
                for h in hs {
                    let (lh, lb, lc) = h.join().unwrap();
                    for k in 0..5 { hits[si][k] += lh[k]; tot_b[k] += lb[k]; tot_c[k] += lc[k]; }
                }
            });
        }
        let b50 = fit50(&snrs, &(0..snrs.len()).map(|s| hits[s][0]).collect::<Vec<_>>(), trials);
        println!("\n--- {name} (full-sweep gains/losses across all {} trial-minutes) ---", trials * snrs.len());
        for (k, &(cname, _)) in cfgs.iter().enumerate() {
            let x50 = fit50(&snrs, &(0..snrs.len()).map(|s| hits[s][k]).collect::<Vec<_>>(), trials);
            println!("  {:<13}: SNR50 = {:+6.2} dB (d={:+5.2} dB) | full-sweep lost={:>2}, gained={:>3}", cname, x50, x50 - b50, tot_b[k], tot_c[k]);
        }
    }

    // Standalone hyg24_full 1,000-minute noise-only FAR
    let chunk = n_mins.div_ceil(nt);
    let mut fa = [0usize; 5];
    thread::scope(|s| {
        let mut hs = Vec::new();
        for ti in 0..nt {
            let st = ti * chunk; let en = (st + chunk).min(n_mins);
            if st >= en { continue; }
            hs.push(s.spawn(move || {
                let mut rx = Rx512::new();
                let mut lfa = [0usize; 5];
                for i in st..en {
                    let nz = noise(4_000_000 + i as u64);
                    rx.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 { rx.start_subslot(); }
                        let base = sub * RATE as usize * 10;
                        for blk in nz[base..base + RATE as usize * 10].chunks(BLOCK) { rx.push_samples(blk); }
                    }
                    for (k, &(_, cfg)) in cfgs.iter().enumerate() {
                        let mut out = [Decoded::EMPTY; MAX_DECODED];
                        lfa[k] += rx.finish_with(cfg, &mut out);
                    }
                }
                lfa
            }));
        }
        for h in hs {
            let l = h.join().unwrap();
            for k in 0..5 { fa[k] += l[k]; }
        }
    });
    println!("\n=== STANDALONE NOISE-ONLY FAR ({n_mins} independent minutes) ===");
    for (k, &(cname, _)) in cfgs.iter().enumerate() {
        println!("  {:<13}: {}/{n_mins}", cname, fa[k]);
    }
}
