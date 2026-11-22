//! Step 3 Prototype 2 & 3 on 512-pt baseline (0 extra static RAM):
//!   - `base`:        current CompactReceiver60 (MAX_CAND=10, same-sym ±3 dedup, point compact)
//!   - `hyg24`:       shortlist hygiene (MAX_CAND=24, |d_sym|<=1 & |df|<=4 dedup, plateau center)
//!   - `pool1`:       same-size ±1-bin tone pooling in compact table + ±2 half-bin compact sync
//!   - `hyg24+pool1`: both combined
//!
//! Usage:
//!   cargo run --release --example proto_512_hygiene_pool -- [trials_per_snr] [noise_minutes]

use std::thread;
use urad950::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use urad950::ldpc::{self, BpScratch, N_BITS};
use urad950::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS, S0, S1, S2, SUBCHANNELS_HZ};
use urad950::rx::{FFT_M, FFT_N, FIRST_BIN, N_BINS, SYM_SAMPLES};
use urad950::rx60::FOLD_SYMS;
use urad950::{
    encode_tones, Callsign, CompactReceiver60, Decoded, Frame, Modulator, DEFAULT_SPACING_HZ,
    MAX_DECODED,
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

struct Rx512Proto {
    hygiene: bool,
    pool1: bool,
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: usize,
    decim_count: u32,
    fold: [[u8; N_BINS]; FOLD_SYMS],
    compact: [[[u8; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
}

impl Rx512Proto {
    fn new(hygiene: bool, pool1: bool) -> Self {
        Self {
            hygiene,
            pool1,
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
        for row in &mut self.fold {
            row.fill(0);
        }
        for ch in &mut self.compact {
            for sym in ch {
                sym.fill(0);
            }
        }
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
                if self.pcm_pos == SYM_SAMPLES {
                    self.pcm_pos = 0;
                }
                self.decim_count = self.decim_count.wrapping_add(1);
                if self.decim_count.is_multiple_of(SYM_SAMPLES as u32) {
                    let sym = (self.decim_count / SYM_SAMPLES as u32) as usize - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol(sym);
                    }
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
        let steady_n = SYM_SAMPLES - RAMP_SAMPLES_6K;
        for sample in z.iter_mut().take(steady_n) {
            *sample = self.pcm_ring[idx];
            idx = if idx + 1 == SYM_SAMPLES { 0 } else { idx + 1 };
        }
        let shift = cfft(&mut z, FFT_M);

        let mut cur_bins = [0u8; N_BINS];
        let mut out_i = 0usize;
        let mut sum = 0u32;
        let mut peak = 0u8;
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            let v = lp.clamp(0, 255) as u8;
            cur_bins[out_i] = v;
            sum += v as u32;
            if v > peak {
                peak = v;
            }
            out_i += 1;
        });
        let floor = (sum / N_BINS as u32) as i32;
        let peak_span = (i32::from(peak) - floor).clamp(18, 96);
        for b in 0..N_BINS {
            let q = Self::quantize(i32::from(cur_bins[b]), floor, peak_span) as u8;
            self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
        }

        // Compact table accumulation
        if sym >= COMPACT_FIRST_SYM && sym <= COMPACT_LAST_SYM {
            let mut histogram = [0u8; 32];
            for &b in &cur_bins {
                histogram[usize::from(b >> 3)] += 1;
            }
            let mut seen = 0usize;
            let mut upper = 0i32;
            for (bucket, &count) in histogram.iter().enumerate().rev() {
                seen += usize::from(count);
                if seen > N_BINS / 20 {
                    upper = (bucket * 8 + 7) as i32;
                    break;
                }
            }
            let local_span = (upper - floor).clamp(18, 96);
            if local_span < peak_span {
                let c_sym = sym - COMPACT_FIRST_SYM;
                for ch in 0..COMPACT_CHANNELS {
                    for tone in 0..4usize {
                        let pos = NOMINAL_HALF_BINS[ch] * 8 + tone * TONE_SPACING_Q4;
                        let added = if self.pool1 {
                            let center = (pos + 8) >> 4;
                            let lo = center.saturating_sub(1);
                            let hi = (center + 1).min(N_BINS - 1);
                            let mut mx = 0u8;
                            for b in lo..=hi {
                                mx = mx.max(cur_bins[b]);
                            }
                            let old_q = Self::quantize(i32::from(mx), floor, peak_span);
                            let loc_q = Self::quantize(i32::from(mx), floor, local_span);
                            (loc_q - old_q).max(0) as u8
                        } else {
                            let bin = pos >> 4;
                            let frac = (pos & 15) as i32;
                            let old_q4 = Self::quantize(i32::from(cur_bins[bin]), floor, peak_span)
                                * (16 - frac)
                                + Self::quantize(i32::from(cur_bins[bin + 1]), floor, peak_span)
                                    * frac;
                            let loc_q4 = Self::quantize(i32::from(cur_bins[bin]), floor, local_span)
                                * (16 - frac)
                                + Self::quantize(i32::from(cur_bins[bin + 1]), floor, local_span)
                                    * frac;
                            ((loc_q4 - old_q4 + 8) >> 4).max(0) as u8
                        };
                        if added > 0 {
                            self.compact[ch][c_sym][tone] =
                                self.compact[ch][c_sym][tone].saturating_add(added).min(7);
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
        if b + 1 >= N_BINS || sym >= FOLD_SYMS {
            return 0;
        }
        let v0 = i32::from(self.fold[sym][b]);
        let v1 = i32::from(self.fold[sym][b + 1]);
        v0 * (16 - frac) + v1 * frac
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
                let base = if self.pool1 {
                    let pos = f0_half * 8 + t * TONE_SPACING_Q4;
                    let center = (pos + 8) >> 4;
                    let lo = center.saturating_sub(1);
                    let hi = (center + 1).min(N_BINS - 1);
                    let mut mx = 0u8;
                    for b in lo..=hi {
                        mx = mx.max(self.fold[s][b]);
                    }
                    i32::from(mx) * 16
                } else {
                    self.sample_tone_q4(s, f0_half, t as u8)
                };
                base + i32::from(self.compact[ch][c_sym][t]) * 16
            });
            interleaved[2 * d] =
                (tones[0].max(tones[1]) - tones[2].max(tones[3])).clamp(-127, 127) as i8;
            interleaved[2 * d + 1] =
                (tones[0].max(tones[3]) - tones[1].max(tones[2])).clamp(-127, 127) as i8;
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

    fn finish_minute(&mut self, out: &mut [Decoded; MAX_DECODED]) -> (usize, usize) {
        let max_cand = if self.hygiene { 24 } else { 10 };
        let mut best: Vec<(i32, u8, u16)> = vec![(i32::MIN, 0, 0); max_cand];
        let mut n_best = 0usize;
        let max_f0_half = ((N_BINS - 1) * 16 - 1 - 3 * TONE_SPACING_Q4) / HALF_BIN_Q4;
        let mut compact_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];

        for start_sym in 1usize..=(FOLD_SYMS - 110) {
            // Track per-channel best sync score across nominal ±2 half-bins when pool1 is enabled
            let mut ch_best_score = [i32::MIN; COMPACT_CHANNELS];
            // For plateau center tie-break (when hygiene is enabled), buffer row scores
            let mut row_scores = [0i32; 320];

            for f0_half in 0..=max_f0_half {
                let mut score = 0i32;
                for (k, &t) in S0.iter().enumerate() {
                    let s = start_sym + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0)
                        + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2)
                        + self.sample_tone_q4(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S1.iter().enumerate() {
                    let s = start_sym + 18 + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0)
                        + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2)
                        + self.sample_tone_q4(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S2.iter().enumerate() {
                    let s = start_sym + 104 + k;
                    let sig = self.sample_tone_q4(s, f0_half, t);
                    let bg = (self.sample_tone_q4(s, f0_half, 0)
                        + self.sample_tone_q4(s, f0_half, 1)
                        + self.sample_tone_q4(s, f0_half, 2)
                        + self.sample_tone_q4(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }
                row_scores[f0_half] = score;

                if self.pool1 {
                    for ch in 0..COMPACT_CHANNELS {
                        if f0_half.abs_diff(NOMINAL_HALF_BINS[ch]) <= 2 {
                            ch_best_score[ch] = ch_best_score[ch].max(score);
                        }
                    }
                } else {
                    for ch in 0..COMPACT_CHANNELS {
                        if f0_half == NOMINAL_HALF_BINS[ch] {
                            ch_best_score[ch] = score;
                        }
                    }
                }
            }

            // Update compact_sync[ch] for this start_sym
            for ch in 0..COMPACT_CHANNELS {
                let sc = ch_best_score[ch];
                let bs = &mut compact_sync[ch];
                if sc > bs[0].0 {
                    bs[1] = bs[0];
                    bs[0] = (sc, start_sym as u8);
                } else if sc > bs[1].0 {
                    bs[1] = (sc, start_sym as u8);
                }
            }

            // Insert candidates from row_scores into `best`
            let mut f0_half = 0usize;
            while f0_half <= max_f0_half {
                let score = row_scores[f0_half];
                let mut picked_half = f0_half;
                if self.hygiene {
                    // Collapse flat plateau to its center
                    let mut end = f0_half;
                    while end + 1 <= max_f0_half && row_scores[end + 1] == score {
                        end += 1;
                    }
                    picked_half = (f0_half + end) / 2;
                    f0_half = end + 1;
                } else {
                    f0_half += 1;
                }

                if n_best == max_cand && score <= best[max_cand - 1].0 {
                    continue;
                }
                let mut dup_idx = None;
                for (i, item) in best.iter().enumerate().take(n_best) {
                    let sym_match = if self.hygiene {
                        item.1.abs_diff(start_sym as u8) <= 1
                    } else {
                        item.1 == start_sym as u8
                    };
                    let df_tol = if self.hygiene { 4 } else { 3 };
                    if sym_match && (item.2 as usize).abs_diff(picked_half) <= df_tol {
                        dup_idx = Some(i);
                        break;
                    }
                }
                if let Some(di) = dup_idx {
                    if score > best[di].0 {
                        best[di] = (score, start_sym as u8, picked_half as u16);
                    }
                } else if n_best < max_cand {
                    best[n_best] = (score, start_sym as u8, picked_half as u16);
                    n_best += 1;
                } else {
                    best[max_cand - 1] = (score, start_sym as u8, picked_half as u16);
                }
                for i in 1..n_best {
                    let mut j = i;
                    while j > 0 && best[j].0 > best[j - 1].0 {
                        best.swap(j, j - 1);
                        j -= 1;
                    }
                }
            }
        }

        let mut scratch = BpScratch::new();
        let mut n_out = 0usize;
        for &(score, start_sym, f0_half) in best.iter().take(n_best) {
            if n_out >= MAX_DECODED {
                break;
            }
            if let Some(frame) = self.decode_primary(start_sym, f0_half, &mut scratch) {
                let packed = frame.pack_u64();
                if out[..n_out].iter().any(|d| d.frame.pack_u64() == packed) {
                    continue;
                }
                let audio_hz = (((FIRST_BIN as u32 * 2 + f0_half as u32) * 6000) / 1024) as u16;
                let time_offset_ms = (start_sym as i16 - 5) * 80;
                out[n_out] = Decoded {
                    frame,
                    audio_hz,
                    time_offset_ms,
                    snr_db: (-22 + score / 180).clamp(-24, 10) as i8,
                };
                n_out += 1;
            }
        }
        let primary_count = n_out;
        for rank in 0..2 {
            for (ch, starts) in compact_sync.iter().enumerate() {
                if n_out == MAX_DECODED {
                    break;
                }
                let (score, start_sym) = starts[rank];
                let time_offset_ms = (start_sym as i16 - 5) * 80;
                if out[..n_out].iter().any(|d| {
                    d.audio_hz.abs_diff(SUBCHANNELS_HZ[ch]) <= 18
                        && d.time_offset_ms == time_offset_ms
                }) {
                    continue;
                }
                if let Some(frame) = self.decode_compact(ch, start_sym, &mut scratch) {
                    if !out[..n_out].iter().any(|d| d.frame == frame) {
                        out[n_out] = Decoded {
                            frame,
                            audio_hz: SUBCHANNELS_HZ[ch],
                            time_offset_ms,
                            snr_db: (-22 + score / 180).clamp(-24, 10) as i8,
                        };
                        n_out += 1;
                    }
                }
            }
        }
        (primary_count, n_out)
    }
}

fn cq(call: &[u8], grid: u16) -> Frame {
    Frame::Cq {
        sender: Callsign::encode(call).unwrap(),
        grid4: grid,
        tx_pwr: 10,
        reserved: 0,
    }
}

fn synth_subslot_wave(frame: Frame, hz: u32, db: f64, offset_ms: usize) -> Vec<i16> {
    let noise_power = SIGMA.powi(2) * 2500.0 / (RATE / 2) as f64;
    let amp = (2.0 * noise_power * 10f64.powf(db / 10.0))
        .sqrt()
        .round()
        .max(1.0) as i16;
    let mut wave = vec![0i16; RATE as usize * 10];
    let mut tx = Modulator::new(encode_tones(frame), RATE, hz, DEFAULT_SPACING_HZ, amp);
    for sample in &mut wave[offset_ms * 24..] {
        let Some(v) = tx.next_sample() else { break };
        *sample = v;
    }
    wave
}

fn synth_noise_minute(seed: u64) -> Vec<i16> {
    let mut state = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ 0xA5A5_5A5A_1234_5678;
    if state == 0 {
        state = 1;
    }
    (0..RATE as usize * 10 * REPEATS)
        .map(|_| {
            let mut sum = -6.0;
            for _ in 0..12 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                sum += (state >> 16) as u32 as f64 / u32::MAX as f64;
            }
            (sum * SIGMA).round().clamp(-32_000.0, 32_000.0) as i16
        })
        .collect()
}

fn fit_logistic_50(snrs: &[f64], hits: &[usize], trials: &[usize]) -> Option<f64> {
    let mut a = 22.0f64;
    let mut b = 1.0f64;
    for _ in 0..40 {
        let (mut g0, mut g1, mut h00, mut h01, mut h11) = (0.0, 0.0, 1e-6, 0.0, 1e-6);
        for ((&x, &k), &n) in snrs.iter().zip(hits).zip(trials) {
            let nf = n as f64;
            let kf = k as f64;
            let z = (a + b * x).clamp(-30.0, 30.0);
            let p = 1.0 / (1.0 + (-z).exp());
            let w = nf * p * (1.0 - p);
            g0 += kf - nf * p;
            g1 += (kf - nf * p) * x;
            h00 += w;
            h01 += w * x;
            h11 += w * x * x;
        }
        let det = h00 * h11 - h01 * h01;
        if det.abs() < 1e-12 {
            break;
        }
        let da = (h11 * g0 - h01 * g1) / det;
        let db = (-h01 * g0 + h00 * g1) / det;
        a += da;
        b += db;
        if da.abs() + db.abs() < 1e-7 {
            break;
        }
    }
    if b <= 1e-4 {
        None
    } else {
        Some(-a / b)
    }
}

fn mcnemar_p(b: usize, c: usize) -> f64 {
    let n = b + c;
    if n == 0 {
        return 1.0;
    }
    let k = b.min(c);
    let mut lf = vec![0.0f64; n + 1];
    for i in 1..=n {
        lf[i] = lf[i - 1] + (i as f64).ln();
    }
    let ln2 = 2.0f64.ln();
    let mut sum = 0.0;
    for i in 0..=k {
        sum += (lf[n] - lf[i] - lf[n - i] - (n as f64) * ln2).exp();
    }
    (2.0 * sum).min(1.0)
}

fn run_sweep(name: &str, hz: u32, nbr_hz: Option<u32>, snrs: &[f64], trials: usize, base_seed: u64) {
    let target = cq(b"NQ6N", 1234);
    let neighbor = cq(b"K1ABC", 2345);
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    let mut hits = vec![[0usize; 4]; snrs.len()];
    let mut unexp = [0usize; 4];
    let mut outcomes: Vec<Vec<[bool; 4]>> = Vec::with_capacity(snrs.len());

    for (s_idx, &snr_db) in snrs.iter().enumerate() {
        let tw = synth_subslot_wave(target, hz, snr_db, 400);
        let nw = nbr_hz.map(|nh| synth_subslot_wave(neighbor, nh, snr_db + 18.0, 440));
        let chunk = trials.div_ceil(n_threads);
        let mut res = Vec::with_capacity(trials);

        thread::scope(|s| {
            let mut handles = Vec::new();
            for ti in 0..n_threads {
                let start = ti * chunk;
                let end = (start + chunk).min(trials);
                if start >= end {
                    continue;
                }
                let tw_r = &tw;
                let nw_r = &nw;
                handles.push(s.spawn(move || {
                    let mut loc = Vec::with_capacity(end - start);
                    let mut loc_u = [0usize; 4];
                    let mut rx0 = CompactReceiver60::new();
                    let mut rx1 = Rx512Proto::new(true, false);
                    let mut rx2 = Rx512Proto::new(false, true);
                    let mut rx3 = Rx512Proto::new(true, true);

                    for i in start..end {
                        let seed = base_seed
                            .wrapping_add((s_idx as u64) * 10_000)
                            .wrapping_add(i as u64);
                        let noise_min = synth_noise_minute(seed);
                        rx0.reset();
                        rx1.reset();
                        rx2.reset();
                        rx3.reset();
                        for sub in 0..REPEATS {
                            if sub > 0 {
                                rx0.start_subslot();
                                rx1.start_subslot();
                                rx2.start_subslot();
                                rx3.start_subslot();
                            }
                            let base = sub * tw_r.len();
                            for blk in 0..(tw_r.len() / BLOCK) {
                                let off = blk * BLOCK;
                                let samples: [i16; BLOCK] = core::array::from_fn(|k| {
                                    let mut a = i32::from(noise_min[base + off + k])
                                        + i32::from(tw_r[off + k]);
                                    if let Some(n) = nw_r {
                                        a += i32::from(n[off + k]);
                                    }
                                    a.clamp(-32_768, 32_767) as i16
                                });
                                rx0.push_samples(&samples);
                                rx1.push_samples(&samples);
                                rx2.push_samples(&samples);
                                rx3.push_samples(&samples);
                            }
                        }
                        let mut o0 = [Decoded::EMPTY; MAX_DECODED];
                        let mut o1 = [Decoded::EMPTY; MAX_DECODED];
                        let mut o2 = [Decoded::EMPTY; MAX_DECODED];
                        let mut o3 = [Decoded::EMPTY; MAX_DECODED];
                        let n0 = rx0.finish_minute(&mut o0);
                        let n1 = rx1.finish_minute(&mut o1).1;
                        let n2 = rx2.finish_minute(&mut o2).1;
                        let n3 = rx3.finish_minute(&mut o3).1;

                        for (idx, (o, n)) in [(&o0, n0), (&o1, n1), (&o2, n2), (&o3, n3)]
                            .into_iter()
                            .enumerate()
                        {
                            loc_u[idx] += o[..n]
                                .iter()
                                .filter(|d| d.frame != target && d.frame != neighbor)
                                .count();
                        }
                        loc.push([
                            o0[..n0].iter().any(|d| d.frame == target),
                            o1[..n1].iter().any(|d| d.frame == target),
                            o2[..n2].iter().any(|d| d.frame == target),
                            o3[..n3].iter().any(|d| d.frame == target),
                        ]);
                    }
                    (loc, loc_u)
                }));
            }
            for h in handles {
                let (l, lu) = h.join().unwrap();
                res.extend(l);
                for k in 0..4 {
                    unexp[k] += lu[k];
                }
            }
        });

        for r in &res {
            for k in 0..4 {
                if r[k] {
                    hits[s_idx][k] += 1;
                }
            }
        }
        outcomes.push(res);
    }

    let tvec = vec![trials; snrs.len()];
    let s50: [Option<f64>; 4] = core::array::from_fn(|k| {
        let hv: Vec<usize> = (0..snrs.len()).map(|s| hits[s][k]).collect();
        fit_logistic_50(snrs, &hv, &tvec)
    });
    let base50 = s50[0].unwrap_or(-21.0);
    let knee = snrs
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| ((*a - base50).abs()).total_cmp(&((*b - base50).abs())))
        .unwrap()
        .0;

    let names = ["base", "hyg24", "pool1", "hyg24+pool1"];
    println!("\n--- {name} ({trials} trials/SNR, knee={:+.1} dB) ---", snrs[knee]);
    for k in 0..4 {
        let x50 = s50[k].unwrap_or(f64::NAN);
        let diff = x50 - base50;
        let (mut a_only, mut b_only) = (0, 0);
        for r in &outcomes[knee] {
            if r[0] && !r[k] {
                a_only += 1;
            } else if !r[0] && r[k] {
                b_only += 1;
            }
        }
        let p = if k == 0 { 1.0 } else { mcnemar_p(a_only, b_only) };
        println!(
            "  {:<12}: SNR50 = {:+6.2} dB (d={:+5.2} dB) | knee {:>3}/{:<3} (base_only={:>2}, cand_only={:>2}, p={:.4}) | unexp={}",
            names[k], x50, diff, hits[knee][k], trials, a_only, b_only, p, unexp[k]
        );
    }
}

fn run_noise_far(minutes: usize) {
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    let chunk = minutes.div_ceil(n_threads);
    let mut fa = [0usize; 2];

    thread::scope(|s| {
        let mut handles = Vec::new();
        for ti in 0..n_threads {
            let start = ti * chunk;
            let end = (start + chunk).min(minutes);
            if start >= end {
                continue;
            }
            handles.push(s.spawn(move || {
                let mut lfa = [0usize; 2];
                let mut rx0 = CompactReceiver60::new();
                let mut rx3 = Rx512Proto::new(true, true);
                for i in start..end {
                    let noise_min = synth_noise_minute(900_000 + i as u64);
                    rx0.reset();
                    rx3.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx0.start_subslot();
                            rx3.start_subslot();
                        }
                        let base = sub * RATE as usize * 10;
                        for blk in noise_min[base..base + RATE as usize * 10].chunks(BLOCK) {
                            rx0.push_samples(blk);
                            rx3.push_samples(blk);
                        }
                    }
                    let mut o0 = [Decoded::EMPTY; MAX_DECODED];
                    let mut o3 = [Decoded::EMPTY; MAX_DECODED];
                    lfa[0] += rx0.finish_minute(&mut o0);
                    lfa[1] += rx3.finish_minute(&mut o3).1;
                }
                lfa
            }));
        }
        for h in handles {
            let l = h.join().unwrap();
            fa[0] += l[0];
            fa[1] += l[1];
        }
    });
    println!(
        "\n=== NOISE-ONLY FAR ({minutes} independent minutes) ===\n  base: {}\n  hyg24+pool1: {}",
        fa[0], fa[1]
    );
}

fn main() {
    let trials: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(200);
    let noise_mins: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);

    let iso_snrs = [-23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0];
    let nbr_snrs = [-22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];

    println!("=== 512-pt SHORTLIST HYGIENE + ±1-BIN COMPACT POOLING ({trials} trials/SNR) ===");
    run_sweep("iso_1275_0Hz", 1275, None, &iso_snrs, trials, 700_000);
    run_sweep("iso_2175_0Hz", 2175, None, &iso_snrs, trials, 710_000);
    run_sweep("nbr18_1275_0Hz", 1275, Some(1050), &nbr_snrs, trials, 720_000);
    run_sweep("nbr18_2175_0Hz", 2175, Some(1950), &nbr_snrs, trials, 730_000);
    run_sweep("nbr18_1275_+5Hz", 1280, Some(1050), &nbr_snrs, trials, 740_000);
    run_sweep("nbr18_1275_-5Hz", 1270, Some(1050), &nbr_snrs, trials, 750_000);
    run_sweep("nbr18_1275_+10Hz", 1285, Some(1050), &nbr_snrs, trials, 760_000);
    run_sweep("nbr18_1275_-10Hz", 1265, Some(1050), &nbr_snrs, trials, 770_000);
    run_sweep("nbr18_2175_+5Hz", 2180, Some(1950), &nbr_snrs, trials, 780_000);
    run_sweep("nbr18_2175_-5Hz", 2170, Some(1950), &nbr_snrs, trials, 790_000);
    run_sweep("nbr18_2175_-10Hz", 2165, Some(1950), &nbr_snrs, trials, 800_000);
    run_noise_far(noise_mins);
}
