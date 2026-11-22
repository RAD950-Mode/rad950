//! Step 3 Prototypes:
//!   A. Single-fold local/fixed-span normalization (replacing global peak-span + 1,236 B compact table)
//!      - Measures BOTH weak-target SNR50 AND strong-neighbor recovery (at -3 dB and +10 dB!)
//!      - Tests isolated controls, +18 dB neighbor (0, +5, +10, -10 Hz), crowded 5-caller,
//!        impulse subslot, and 1,000-minute noise-only FAR!
//!   B. Half-symbol timing sweep (offset_ms = 400, 410, 420, 430, 440 ms) comparing single-phase
//!      vs dual-phase (0 ms + 40 ms) FFT framing.
//!
//! Usage:
//!   cargo run --release --example proto_localnorm_and_halfsym -- [trials_per_snr] [noise_mins]

use std::thread;
use urad950::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use urad950::ldpc::{self, BpScratch, N_BITS};
use urad950::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS, S0, S1, S2};
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

static LP_24K_TO_6K: [i16; 31] = lowpass::<31>(2_550.0 / 24_000.0);

#[derive(Clone, Copy)]
enum NormMode {
    /// Global median-ish (40th percentile) floor + fixed span (e.g. 32 codes = 12 dB) — single fold, 0 compact bytes!
    GlobalP40FixedSpan(i32),
    /// Smoothly blended local floor (sliding 31-bin trimmed mean) + fixed span — single fold, 0 compact bytes!
    LocalSmoothFixedSpan(i32),
    /// Smoothly blended local floor + local 85th-percentile span — single fold, 0 compact bytes!
    LocalSmoothP85,
}

/// Single-fold receiver (10,875 B fold, 0 B compact table) with configurable normalization and
/// optional half-symbol (+40 ms) phase offset.
struct RxSingleFold {
    mode: NormMode,
    phase_offset_samples: usize, // 0 for standard, 240 (40 ms @ 6 kHz) for half-symbol phase
    hygiene: bool,
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: usize,
    decim_count: u32,
    fold: [[u8; N_BINS]; FOLD_SYMS],
}

impl RxSingleFold {
    fn new(mode: NormMode, phase_offset_samples: usize, hygiene: bool) -> Self {
        Self {
            mode,
            phase_offset_samples,
            hygiene,
            decim: Decimator::new(),
            pcm_ring: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            fold: [[0; N_BINS]; FOLD_SYMS],
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
                let dc = self.decim_count as usize;
                if dc >= SYM_SAMPLES + self.phase_offset_samples
                    && (dc - self.phase_offset_samples).is_multiple_of(SYM_SAMPLES)
                {
                    let sym = (dc - self.phase_offset_samples) / SYM_SAMPLES - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol(sym);
                    }
                }
            }
        }
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
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            cur_bins[out_i] = lp.clamp(0, 255) as u8;
            out_i += 1;
        });

        match self.mode {
            NormMode::GlobalP40FixedSpan(span) => {
                // 40th percentile floor from 32-bucket histogram (robust against even 50% crowded passband!)
                let mut hist = [0u8; 32];
                for &v in &cur_bins {
                    hist[usize::from(v >> 3)] += 1;
                }
                let target_cnt = (N_BINS * 40) / 100;
                let mut seen = 0usize;
                let mut floor = 0i32;
                for (bucket, &c) in hist.iter().enumerate() {
                    seen += usize::from(c);
                    if seen >= target_cnt {
                        floor = (bucket * 8 + 4) as i32;
                        break;
                    }
                }
                for b in 0..N_BINS {
                    let q = ((i32::from(cur_bins[b]) - floor + 3).max(0) * 6 / span).clamp(0, 6)
                        as u8;
                    self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
                }
            }
            NormMode::LocalSmoothFixedSpan(span) => {
                // Smoothly blended local floor: 6 overlapping bands of 25 bins, linearly interpolated
                // between band centers using each band's 40th percentile.
                const NB: usize = 6;
                const BW: usize = N_BINS / NB; // 25 bins (~293 Hz)
                let mut band_floor = [0i32; NB];
                for k in 0..NB {
                    let mut tmp = [0u8; BW];
                    tmp.copy_from_slice(&cur_bins[k * BW..(k + 1) * BW]);
                    tmp.sort_unstable();
                    band_floor[k] = i32::from(tmp[BW * 40 / 100]);
                }
                for b in 0..N_BINS {
                    // Linearly interpolate between nearest band centers (center of band k is k*25 + 12)
                    let pos = b as i32 - 12;
                    let floor = if pos <= 0 {
                        band_floor[0]
                    } else if pos >= ((NB - 1) * BW) as i32 {
                        band_floor[NB - 1]
                    } else {
                        let k = (pos as usize) / BW;
                        let rem = (pos as usize) % BW;
                        (band_floor[k] * (BW - rem) as i32 + band_floor[k + 1] * rem as i32)
                            / BW as i32
                    };
                    let q = ((i32::from(cur_bins[b]) - floor + 3).max(0) * 6 / span).clamp(0, 6)
                        as u8;
                    self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
                }
            }
            NormMode::LocalSmoothP85 => {
                const NB: usize = 6;
                const BW: usize = N_BINS / NB;
                let mut band_floor = [0i32; NB];
                let mut band_span = [0i32; NB];
                for k in 0..NB {
                    let mut tmp = [0u8; BW];
                    tmp.copy_from_slice(&cur_bins[k * BW..(k + 1) * BW]);
                    tmp.sort_unstable();
                    let fl = i32::from(tmp[BW * 40 / 100]);
                    let hi = i32::from(tmp[BW * 85 / 100]);
                    band_floor[k] = fl;
                    band_span[k] = ((hi - fl) * 2).clamp(24, 96);
                }
                for b in 0..N_BINS {
                    let pos = b as i32 - 12;
                    let (floor, span) = if pos <= 0 {
                        (band_floor[0], band_span[0])
                    } else if pos >= ((NB - 1) * BW) as i32 {
                        (band_floor[NB - 1], band_span[NB - 1])
                    } else {
                        let k = (pos as usize) / BW;
                        let rem = (pos as usize) % BW;
                        let fl = (band_floor[k] * (BW - rem) as i32
                            + band_floor[k + 1] * rem as i32)
                            / BW as i32;
                        let sp = (band_span[k] * (BW - rem) as i32 + band_span[k + 1] * rem as i32)
                            / BW as i32;
                        (fl, sp)
                    };
                    let q = ((i32::from(cur_bins[b]) - floor + 3).max(0) * 6 / span).clamp(0, 6)
                        as u8;
                    self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
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

    fn finish_minute(&mut self, out: &mut [Decoded; MAX_DECODED]) -> usize {
        let max_cand = if self.hygiene { 24 } else { 10 };
        let mut best: Vec<(i32, u8, u16)> = vec![(i32::MIN, 0, 0); max_cand];
        let mut n_best = 0usize;
        let max_f0_half = ((N_BINS - 1) * 16 - 1 - 3 * TONE_SPACING_Q4) / HALF_BIN_Q4;

        for start_sym in 1usize..=(FOLD_SYMS - 110) {
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
            }

            let mut f0_half = 0usize;
            while f0_half <= max_f0_half {
                let score = row_scores[f0_half];
                let mut picked_half = f0_half;
                if self.hygiene {
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
        n_out
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

fn run_localnorm_sweep(
    name: &str,
    hz: u32,
    nbr: Option<(u32, f64, usize)>, // (nbr_hz, delta_db, offset_ms)
    snrs: &[f64],
    trials: usize,
    base_seed: u64,
) {
    let target = cq(b"NQ6N", 1234);
    let neighbor = cq(b"K1ABC", 2345);
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    // 4 receivers:
    //   0: base_compact (CompactReceiver60, 12,111 B fold+compact)
    //   1: glob_p40_sp32 (single fold, 10,875 B, frees 1,236 B, hygiene=true)
    //   2: loc_sm_sp32   (single fold, 10,875 B, frees 1,236 B, hygiene=true)
    //   3: loc_sm_p85    (single fold, 10,875 B, frees 1,236 B, hygiene=true)
    let mut hits_t = vec![[0usize; 4]; snrs.len()];
    let mut hits_n = vec![[0usize; 4]; snrs.len()];
    let mut unexp = [0usize; 4];
    let mut outcomes: Vec<Vec<[bool; 4]>> = Vec::with_capacity(snrs.len());

    for (s_idx, &snr_db) in snrs.iter().enumerate() {
        let tw = synth_subslot_wave(target, hz, snr_db, 400);
        let nw = nbr.map(|(nh, ddb, oms)| synth_subslot_wave(neighbor, nh, snr_db + ddb, oms));
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
                    let mut loc_t = Vec::with_capacity(end - start);
                    let mut loc_n = [0usize; 4];
                    let mut loc_u = [0usize; 4];
                    let mut rx0 = CompactReceiver60::new();
                    let mut rx1 = RxSingleFold::new(NormMode::GlobalP40FixedSpan(32), 0, true);
                    let mut rx2 = RxSingleFold::new(NormMode::LocalSmoothFixedSpan(32), 0, true);
                    let mut rx3 = RxSingleFold::new(NormMode::LocalSmoothP85, 0, true);

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
                        let n1 = rx1.finish_minute(&mut o1);
                        let n2 = rx2.finish_minute(&mut o2);
                        let n3 = rx3.finish_minute(&mut o3);

                        for (idx, (o, n)) in [(&o0, n0), (&o1, n1), (&o2, n2), (&o3, n3)]
                            .into_iter()
                            .enumerate()
                        {
                            if o[..n].iter().any(|d| d.frame == neighbor) {
                                loc_n[idx] += 1;
                            }
                            loc_u[idx] += o[..n]
                                .iter()
                                .filter(|d| d.frame != target && d.frame != neighbor)
                                .count();
                        }
                        loc_t.push([
                            o0[..n0].iter().any(|d| d.frame == target),
                            o1[..n1].iter().any(|d| d.frame == target),
                            o2[..n2].iter().any(|d| d.frame == target),
                            o3[..n3].iter().any(|d| d.frame == target),
                        ]);
                    }
                    (loc_t, loc_n, loc_u)
                }));
            }
            for h in handles {
                let (lt, ln, lu) = h.join().unwrap();
                res.extend(lt);
                for k in 0..4 {
                    hits_n[s_idx][k] += ln[k];
                    unexp[k] += lu[k];
                }
            }
        });

        for r in &res {
            for k in 0..4 {
                if r[k] {
                    hits_t[s_idx][k] += 1;
                }
            }
        }
        outcomes.push(res);
    }

    let tvec = vec![trials; snrs.len()];
    let s50: [Option<f64>; 4] = core::array::from_fn(|k| {
        let hv: Vec<usize> = (0..snrs.len()).map(|s| hits_t[s][k]).collect();
        fit_logistic_50(snrs, &hv, &tvec)
    });
    let base50 = s50[0].unwrap_or(-21.0);
    let knee = snrs
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| ((*a - base50).abs()).total_cmp(&((*b - base50).abs())))
        .unwrap()
        .0;

    let names = ["base_comp", "glob_p40_32", "loc_sm_32", "loc_sm_p85"];
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
        let nbr_str = if nbr.is_some() {
            let ntot: usize = (0..snrs.len()).map(|s| hits_n[s][k]).sum();
            format!(" | nbr={:>4}/{}", ntot, trials * snrs.len())
        } else {
            String::new()
        };
        println!(
            "  {:<12}: SNR50 = {:+6.2} dB (d={:+5.2} dB) | knee {:>3}/{:<3} (b={:>2}, c={:>2}, p={:.4}){} | u={}",
            names[k], x50, diff, hits_t[knee][k], trials, a_only, b_only, p, nbr_str, unexp[k]
        );
    }
}

fn run_crowded_and_strong(trials: usize) {
    let t = cq(b"NQ6N", 1234);
    let n1 = cq(b"K1ABC", 2345);
    let n2 = cq(b"W1AW", 3456);
    let n3 = cq(b"VE3XYZ", 4567);
    let n4 = cq(b"G4ABC", 5678);

    println!("\n=== CROWDED 5-CALLER (weak -20 dB @ 2175 Hz + 4 strong callers at -3..-10 dB, {trials} trials) ===");
    let waves = [
        synth_subslot_wave(n1, 600, -3.0, 400),
        synth_subslot_wave(n2, 1050, -6.0, 440),
        synth_subslot_wave(n3, 1500, -10.0, 360),
        synth_subslot_wave(n4, 1950, -3.0, 440),
        synth_subslot_wave(t, 2175, -20.0, 400),
    ];
    let expected = [n1, n2, n3, n4, t];
    let mut got_weak = [0usize; 4];
    let mut got_strong = [0usize; 4];
    let mut unexp = [0usize; 4];

    for i in 0..trials {
        let noise_min = synth_noise_minute(850_000 + i as u64);
        let mut rx0 = CompactReceiver60::new();
        let mut rx1 = RxSingleFold::new(NormMode::GlobalP40FixedSpan(32), 0, true);
        let mut rx2 = RxSingleFold::new(NormMode::LocalSmoothFixedSpan(32), 0, true);
        let mut rx3 = RxSingleFold::new(NormMode::LocalSmoothP85, 0, true);
        for sub in 0..REPEATS {
            if sub > 0 {
                rx0.start_subslot();
                rx1.start_subslot();
                rx2.start_subslot();
                rx3.start_subslot();
            }
            let base = sub * RATE as usize * 10;
            for blk in 0..(RATE as usize * 10 / BLOCK) {
                let off = blk * BLOCK;
                let samples: [i16; BLOCK] = core::array::from_fn(|k| {
                    let mut a = i32::from(noise_min[base + off + k]);
                    for w in &waves {
                        a += i32::from(w[off + k]);
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
        let n1_c = rx1.finish_minute(&mut o1);
        let n2_c = rx2.finish_minute(&mut o2);
        let n3_c = rx3.finish_minute(&mut o3);
        for (idx, (o, n)) in [(&o0, n0), (&o1, n1_c), (&o2, n2_c), (&o3, n3_c)]
            .into_iter()
            .enumerate()
        {
            if o[..n].iter().any(|d| d.frame == t) {
                got_weak[idx] += 1;
            }
            for &s in &expected[..4] {
                if o[..n].iter().any(|d| d.frame == s) {
                    got_strong[idx] += 1;
                }
            }
            unexp[idx] += o[..n]
                .iter()
                .filter(|d| !expected.contains(&d.frame))
                .count();
        }
    }
    let names = ["base_comp", "glob_p40_32", "loc_sm_32", "loc_sm_p85"];
    for k in 0..4 {
        println!(
            "  {:<12}: weak_2175={:>3}/{:<3} | strong_4={:>4}/{:<4} | unexp={}",
            names[k],
            got_weak[k],
            trials,
            got_strong[k],
            trials * 4,
            unexp[k]
        );
    }
}

fn run_half_symbol_sweep(trials: usize) {
    let target = cq(b"NQ6N", 1234);
    let snrs = [-23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0, -18.5, -18.0];
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    println!("\n=== HALF-SYMBOL TIMING SWEEP (1275 Hz isolated, {trials} trials/SNR) ===");
    println!("  offset_ms | single-phase SNR50 | dual-phase (0+40ms) SNR50 | delta");

    for &oms in &[400usize, 410, 420, 430, 440] {
        let mut h_single = vec![0usize; snrs.len()];
        let mut h_dual = vec![0usize; snrs.len()];
        for (si, &snr_db) in snrs.iter().enumerate() {
            let tw = synth_subslot_wave(target, 1275, snr_db, oms);
            let chunk = trials.div_ceil(n_threads);
            thread::scope(|s| {
                let mut handles = Vec::new();
                for ti in 0..n_threads {
                    let start = ti * chunk;
                    let end = (start + chunk).min(trials);
                    if start >= end {
                        continue;
                    }
                    let tw_r = &tw;
                    handles.push(s.spawn(move || {
                        let (mut ls, mut ld) = (0usize, 0usize);
                        let mut rx_a = RxSingleFold::new(NormMode::GlobalP40FixedSpan(32), 0, true);
                        let mut rx_b =
                            RxSingleFold::new(NormMode::GlobalP40FixedSpan(32), 240, true);
                        for i in start..end {
                            let seed = 950_000
                                + (oms as u64) * 100_000
                                + (si as u64) * 10_000
                                + i as u64;
                            let noise_min = synth_noise_minute(seed);
                            rx_a.reset();
                            rx_b.reset();
                            for sub in 0..REPEATS {
                                if sub > 0 {
                                    rx_a.start_subslot();
                                    rx_b.start_subslot();
                                }
                                let base = sub * tw_r.len();
                                for blk in 0..(tw_r.len() / BLOCK) {
                                    let off = blk * BLOCK;
                                    let samples: [i16; BLOCK] = core::array::from_fn(|k| {
                                        noise_min[base + off + k].saturating_add(tw_r[off + k])
                                    });
                                    rx_a.push_samples(&samples);
                                    rx_b.push_samples(&samples);
                                }
                            }
                            let mut oa = [Decoded::EMPTY; MAX_DECODED];
                            let mut ob = [Decoded::EMPTY; MAX_DECODED];
                            let na = rx_a.finish_minute(&mut oa);
                            let nb = rx_b.finish_minute(&mut ob);
                            let ok_a = oa[..na].iter().any(|d| d.frame == target);
                            let ok_b = ob[..nb].iter().any(|d| d.frame == target);
                            if ok_a {
                                ls += 1;
                            }
                            if ok_a || ok_b {
                                ld += 1;
                            }
                        }
                        (ls, ld)
                    }));
                }
                for h in handles {
                    let (ls, ld) = h.join().unwrap();
                    h_single[si] += ls;
                    h_dual[si] += ld;
                }
            });
        }
        let tvec = vec![trials; snrs.len()];
        let s_50 = fit_logistic_50(&snrs, &h_single, &tvec).unwrap_or(f64::NAN);
        let d_50 = fit_logistic_50(&snrs, &h_dual, &tvec).unwrap_or(f64::NAN);
        println!(
            "  {:>4} ms   |      {:+6.2} dB     |         {:+6.2} dB         | {:+6.2} dB",
            oms,
            s_50,
            d_50,
            d_50 - s_50
        );
    }
}

fn run_noise_far(minutes: usize) {
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    let chunk = minutes.div_ceil(n_threads);
    let mut fa = [0usize; 3];
    thread::scope(|s| {
        let mut handles = Vec::new();
        for ti in 0..n_threads {
            let start = ti * chunk;
            let end = (start + chunk).min(minutes);
            if start >= end {
                continue;
            }
            handles.push(s.spawn(move || {
                let mut lfa = [0usize; 3];
                let mut rx1 = RxSingleFold::new(NormMode::GlobalP40FixedSpan(32), 0, true);
                let mut rx2 = RxSingleFold::new(NormMode::LocalSmoothFixedSpan(32), 0, true);
                let mut rx3 = RxSingleFold::new(NormMode::LocalSmoothP85, 0, true);
                for i in start..end {
                    let noise_min = synth_noise_minute(1_200_000 + i as u64);
                    rx1.reset();
                    rx2.reset();
                    rx3.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx1.start_subslot();
                            rx2.start_subslot();
                            rx3.start_subslot();
                        }
                        let base = sub * RATE as usize * 10;
                        for blk in noise_min[base..base + RATE as usize * 10].chunks(BLOCK) {
                            rx1.push_samples(blk);
                            rx2.push_samples(blk);
                            rx3.push_samples(blk);
                        }
                    }
                    let mut o1 = [Decoded::EMPTY; MAX_DECODED];
                    let mut o2 = [Decoded::EMPTY; MAX_DECODED];
                    let mut o3 = [Decoded::EMPTY; MAX_DECODED];
                    lfa[0] += rx1.finish_minute(&mut o1);
                    lfa[1] += rx2.finish_minute(&mut o2);
                    lfa[2] += rx3.finish_minute(&mut o3);
                }
                lfa
            }));
        }
        for h in handles {
            let l = h.join().unwrap();
            for k in 0..3 {
                fa[k] += l[k];
            }
        }
    });
    println!(
        "\n=== NOISE-ONLY FAR ({minutes} independent minutes) ===\n  glob_p40_32: {}\n  loc_sm_32  : {}\n  loc_sm_p85 : {}",
        fa[0], fa[1], fa[2]
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
    let nbr_snrs = [-22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5];

    println!("=== SINGLE-FOLD LOCAL/FIXED-SPAN NORMALIZATION (frees 1,236 B) ({trials} trials/SNR) ===");
    run_localnorm_sweep("iso_1275_0Hz", 1275, None, &iso_snrs, trials, 10_000);
    run_localnorm_sweep("iso_2175_0Hz", 2175, None, &iso_snrs, trials, 20_000);
    run_localnorm_sweep("nbr18_1275_0Hz", 1275, Some((1050, 18.0, 440)), &nbr_snrs, trials, 30_000);
    run_localnorm_sweep("nbr18_2175_0Hz", 2175, Some((1950, 18.0, 440)), &nbr_snrs, trials, 40_000);
    run_localnorm_sweep("nbr18_2175_aligned", 2175, Some((1950, 18.0, 400)), &nbr_snrs, trials, 50_000);
    run_localnorm_sweep("nbr18_2175_+5Hz", 2180, Some((1950, 18.0, 440)), &nbr_snrs, trials, 60_000);
    run_localnorm_sweep("nbr18_1275_+10Hz", 1285, Some((1050, 18.0, 440)), &nbr_snrs, trials, 70_000);
    run_localnorm_sweep("nbr18_2175_-10Hz", 2165, Some((1950, 18.0, 440)), &nbr_snrs, trials, 80_000);
    run_localnorm_sweep("nbr28_strong+7dB", 2175, Some((1950, 28.0, 440)), &nbr_snrs, trials, 90_000);
    run_crowded_and_strong(64);
    run_half_symbol_sweep(100);
    run_noise_far(noise_mins);
}
