//! Step 3 Prototype:
//!   1. Handheld dual-phase (`0 ms` + `+40 ms`) receiver fitting strictly inside the 13 KiB slot:
//!      - 4-bit primary full-band fold (`8,700 B`, per-burst cap 3, frees 2,175 B)
//!      - existing 1,236 B compact table at `0 ms` phase
//!      - 4-bit nominal-channel `+40 ms` phase fold (`8 ch x 116 sym x 4 tones x 4 bits = 1,856 B`,
//!        normalized with 95th-percentile span so it works beside +18 dB neighbors too!)
//!      Total persistent state ~ 12,845 B (451 B spare inside 13,296 B slot budget).
//!   2. `hyg24` per-piece ablation (MAX_CAND=24 vs adjacent-start suppression vs plateau tie-break)
//!      + standalone 1,000-minute noise-only FAR.
//!   3. Real conducted WAV replay mode (`replay <wav_path> [start_sec]`).
//!
//! Usage:
//!   cargo run --release --example proto_halfsym_handheld -- [trials_per_snr] [noise_mins]
//!   cargo run --release --example proto_halfsym_handheld -- tones
//!   cargo run --release --example proto_halfsym_handheld -- replay <wav_path> [start_sec]

use std::fs::File;
use std::io::Read;
use std::thread;
use urad950::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use urad950::frame::encode_grid4;
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

/// Handheld dual-phase (`0 ms` + `+40 ms`) prototype fitting inside the 13 KiB slot.
///
/// Static RAM budget (packed):
///   - 4-bit primary fold: `116 * 150 * 4 / 8 = 8,700 B`
///   - `0 ms` compact table: `1,236 B`
///   - `+40 ms` 4-bit nominal-channel fold: `8 * 116 * 4 * 4 / 8 = 1,856 B`
///   - decimator + `pcm_ring` (shared!) + misc: `~1,053 B`
///   - Total: `~12,845 B` (`451 B` spare inside `13,296 B`).
struct RxDualPhaseHandheld {
    four_bit_primary: bool,
    enable_half_phase: bool,
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: usize,
    decim_count: u32,
    fold: [[u8; N_BINS]; FOLD_SYMS],
    compact: [[[u8; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
    /// 4-bit per-tone (`0..=15`, cap 3/burst) nominal-channel spectrogram at `+40 ms` phase,
    /// quantized using 95th-percentile span so it works both in AWGN and beside +18 dB neighbors.
    half_nom: [[[u8; 4]; FOLD_SYMS]; COMPACT_CHANNELS],
}

impl RxDualPhaseHandheld {
    fn new(four_bit_primary: bool, enable_half_phase: bool) -> Self {
        Self {
            four_bit_primary,
            enable_half_phase,
            decim: Decimator::new(),
            pcm_ring: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            fold: [[0; N_BINS]; FOLD_SYMS],
            compact: [[[0; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
            half_nom: [[[0; 4]; FOLD_SYMS]; COMPACT_CHANNELS],
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
            for s in ch {
                s.fill(0);
            }
        }
        for ch in &mut self.half_nom {
            for s in ch {
                s.fill(0);
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
                let dc = self.decim_count as usize;
                // Phase 0 (0 ms): fires on audio blocks 0, 10, 20, ...
                if dc.is_multiple_of(SYM_SAMPLES) {
                    let sym = dc / SYM_SAMPLES - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol_phase0(sym);
                    }
                } else if self.enable_half_phase
                    && dc >= SYM_SAMPLES + SYM_SAMPLES / 2
                    && (dc - SYM_SAMPLES / 2).is_multiple_of(SYM_SAMPLES)
                {
                    // Phase 1 (+40 ms): fires on audio blocks 5, 15, 25, ... (40 ms away from Phase 0!)
                    let sym = (dc - SYM_SAMPLES / 2) / SYM_SAMPLES - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol_phase_half(sym);
                    }
                }
            }
        }
    }

    fn compute_bins(&self) -> ([u8; N_BINS], i32, i32, i32) {
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
        let mut hist = [0u8; 32];
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            let v = lp.clamp(0, 255) as u8;
            cur_bins[out_i] = v;
            sum += v as u32;
            if v > peak {
                peak = v;
            }
            hist[usize::from(v >> 3)] += 1;
            out_i += 1;
        });
        let floor = (sum / N_BINS as u32) as i32;
        let peak_span = (i32::from(peak) - floor).clamp(18, 96);

        let mut seen = 0usize;
        let mut upper = 0i32;
        for (bucket, &count) in hist.iter().enumerate().rev() {
            seen += usize::from(count);
            if seen > N_BINS / 20 {
                upper = (bucket * 8 + 7) as i32;
                break;
            }
        }
        let pct_span = (upper - floor).clamp(18, 96);
        (cur_bins, floor, peak_span, pct_span)
    }

    #[inline]
    fn quantize_max(lp: i32, floor: i32, span: i32, qmax: i32) -> i32 {
        ((lp - floor + 3).max(0) * qmax / span).clamp(0, qmax)
    }

    fn on_symbol_phase0(&mut self, sym: usize) {
        let (cur_bins, floor, peak_span, pct_span) = self.compute_bins();
        let (qmax, cell_max) = if self.four_bit_primary { (3, 15u8) } else { (6, 31u8) };
        for b in 0..N_BINS {
            let q = Self::quantize_max(i32::from(cur_bins[b]), floor, peak_span, qmax) as u8;
            self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(cell_max);
        }
        // Compact table at 0 ms
        if sym >= COMPACT_FIRST_SYM && sym <= COMPACT_LAST_SYM && pct_span < peak_span {
            let c_sym = sym - COMPACT_FIRST_SYM;
            for ch in 0..COMPACT_CHANNELS {
                for tone in 0..4usize {
                    let pos = NOMINAL_HALF_BINS[ch] * 8 + tone * TONE_SPACING_Q4;
                    let bin = pos >> 4;
                    let frac = (pos & 15) as i32;
                    let old_q4 = Self::quantize_max(i32::from(cur_bins[bin]), floor, peak_span, qmax)
                        * (16 - frac)
                        + Self::quantize_max(i32::from(cur_bins[bin + 1]), floor, peak_span, qmax)
                            * frac;
                    let loc_q4 = Self::quantize_max(i32::from(cur_bins[bin]), floor, pct_span, qmax)
                        * (16 - frac)
                        + Self::quantize_max(i32::from(cur_bins[bin + 1]), floor, pct_span, qmax)
                            * frac;
                    let added = ((loc_q4 - old_q4 + 8) >> 4).max(0) as u8;
                    if added > 0 {
                        self.compact[ch][c_sym][tone] =
                            self.compact[ch][c_sym][tone].saturating_add(added).min(7);
                    }
                }
            }
        }
    }

    fn on_symbol_phase_half(&mut self, sym: usize) {
        let (cur_bins, floor, _peak_span, pct_span) = self.compute_bins();
        // Store 4-bit (0..=15, cap 3/burst) percentile-normalized tones on the 8 nominal channels!
        for ch in 0..COMPACT_CHANNELS {
            for tone in 0..4usize {
                let pos = NOMINAL_HALF_BINS[ch] * 8 + tone * TONE_SPACING_Q4;
                let bin = pos >> 4;
                let frac = (pos & 15) as i32;
                let lp_q4 = i32::from(cur_bins[bin]) * (16 - frac)
                    + i32::from(cur_bins[bin + 1]) * frac;
                let lp = (lp_q4 + 8) >> 4;
                let q = Self::quantize_max(lp, floor, pct_span, 3) as u8;
                self.half_nom[ch][sym][tone] =
                    self.half_nom[ch][sym][tone].saturating_add(q).min(15);
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
                self.sample_tone_q4(s, f0_half, t as u8)
                    + i32::from(self.compact[ch][c_sym][t]) * 16
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

    fn decode_half_nom(&self, ch: usize, start_sym: u8, scratch: &mut BpScratch) -> Option<Frame> {
        let mut inter_llr = [0i32; N_BITS];
        for d in 0..N_DATA_SYMS {
            let s = usize::from(start_sym) + data_sym_pos(d);
            let t = self.half_nom[ch][s];
            let (m0, m1, m2, m3) = (i32::from(t[0]), i32::from(t[1]), i32::from(t[2]), i32::from(t[3]));
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
        const MAX_CAND: usize = 10;
        let mut best: [(i32, u8, u16); MAX_CAND] = [(i32::MIN, 0, 0); MAX_CAND];
        let mut n_best = 0usize;
        let max_f0_half = ((N_BINS - 1) * 16 - 1 - 3 * TONE_SPACING_Q4) / HALF_BIN_Q4;
        let mut compact_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];

        for start_sym in 1usize..=(FOLD_SYMS - 110) {
            let mut nom_ch = 0usize;
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
                if nom_ch < COMPACT_CHANNELS && f0_half == NOMINAL_HALF_BINS[nom_ch] {
                    let bs = &mut compact_sync[nom_ch];
                    if score > bs[0].0 {
                        bs[1] = bs[0];
                        bs[0] = (score, start_sym as u8);
                    } else if score > bs[1].0 {
                        bs[1] = (score, start_sym as u8);
                    }
                    nom_ch += 1;
                }
                if n_best == MAX_CAND && score <= best[MAX_CAND - 1].0 {
                    continue;
                }
                let mut dup_idx = None;
                for (i, item) in best.iter().enumerate().take(n_best) {
                    if item.1 == start_sym as u8 && (item.2 as usize).abs_diff(f0_half) <= 3 {
                        dup_idx = Some(i);
                        break;
                    }
                }
                if let Some(di) = dup_idx {
                    if score > best[di].0 {
                        best[di] = (score, start_sym as u8, f0_half as u16);
                    }
                } else if n_best < MAX_CAND {
                    best[n_best] = (score, start_sym as u8, f0_half as u16);
                    n_best += 1;
                } else {
                    best[MAX_CAND - 1] = (score, start_sym as u8, f0_half as u16);
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
                if out[..n_out].iter().any(|d| d.frame == frame) {
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
        // Compact pass (0 ms)
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
        // Half-symbol (+40 ms) nominal-channel pass (up to 16 attempts: 8 channels x top-2 starts)
        if self.enable_half_phase {
            let mut half_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];
            for ch in 0..COMPACT_CHANNELS {
                for start_sym in 0usize..=(FOLD_SYMS - 110) {
                    let mut score = 0i32;
                    for (k, &t) in S0.iter().enumerate() {
                        let c = self.half_nom[ch][start_sym + k];
                        let bg = (i32::from(c[0]) + i32::from(c[1]) + i32::from(c[2]) + i32::from(c[3])) >> 2;
                        score += i32::from(c[t as usize]) - bg;
                    }
                    for (k, &t) in S1.iter().enumerate() {
                        let c = self.half_nom[ch][start_sym + 18 + k];
                        let bg = (i32::from(c[0]) + i32::from(c[1]) + i32::from(c[2]) + i32::from(c[3])) >> 2;
                        score += i32::from(c[t as usize]) - bg;
                    }
                    for (k, &t) in S2.iter().enumerate() {
                        let c = self.half_nom[ch][start_sym + 104 + k];
                        let bg = (i32::from(c[0]) + i32::from(c[1]) + i32::from(c[2]) + i32::from(c[3])) >> 2;
                        score += i32::from(c[t as usize]) - bg;
                    }
                    let bs = &mut half_sync[ch];
                    if score > bs[0].0 {
                        bs[1] = bs[0];
                        bs[0] = (score, start_sym as u8);
                    } else if score > bs[1].0 {
                        bs[1] = (score, start_sym as u8);
                    }
                }
            }
            for rank in 0..2 {
                for (ch, starts) in half_sync.iter().enumerate() {
                    if n_out == MAX_DECODED {
                        break;
                    }
                    let (score, start_sym) = starts[rank];
                    if let Some(frame) = self.decode_half_nom(ch, start_sym, &mut scratch) {
                        if !out[..n_out].iter().any(|d| d.frame == frame) {
                            out[n_out] = Decoded {
                                frame,
                                audio_hz: SUBCHANNELS_HZ[ch],
                                time_offset_ms: (start_sym as i16 - 5) * 80 + 40,
                                snr_db: (-22 + score / 12).clamp(-24, 10) as i8,
                            };
                            n_out += 1;
                        }
                    }
                }
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
    if b <= 1e-4 { None } else { Some(-a / b) }
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

fn run_sweep(
    name: &str,
    hz: u32,
    offset_ms: usize,
    nbr: Option<(u32, usize)>,
    snrs: &[f64],
    trials: usize,
    base_seed: u64,
) {
    let target = cq(b"NQ6N", 1234);
    let neighbor = cq(b"WP2Z", 2345);
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    // 3 receivers:
    //   0: base_5bit   (CompactReceiver60: 5-bit fold + 1,236 B compact, 0 ms only)
    //   1: four_bit_0  (4-bit primary + 1,236 B compact, 0 ms only — isolates 4-bit cost!)
    //   2: hh_dual_4b  (4-bit primary + 1,236 B compact + 1,856 B +40ms nominal = 12,845 B!)
    let mut hits = vec![[0usize; 3]; snrs.len()];
    let mut unexp = [0usize; 3];
    let mut outcomes: Vec<Vec<[bool; 3]>> = Vec::with_capacity(snrs.len());

    for (si, &snr_db) in snrs.iter().enumerate() {
        let tw = synth_subslot_wave(target, hz, snr_db, offset_ms);
        let nw = nbr.map(|(nh, noms)| synth_subslot_wave(neighbor, nh, snr_db + 18.0, noms));
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
                    let mut lu = [0usize; 3];
                    let mut rx0 = CompactReceiver60::new();
                    let mut rx1 = RxDualPhaseHandheld::new(true, false);
                    let mut rx2 = RxDualPhaseHandheld::new(true, true);

                    for i in start..end {
                        let seed = base_seed + (si as u64) * 10_000 + i as u64;
                        let noise_min = synth_noise_minute(seed);
                        rx0.reset();
                        rx1.reset();
                        rx2.reset();
                        for sub in 0..REPEATS {
                            if sub > 0 {
                                rx0.start_subslot();
                                rx1.start_subslot();
                                rx2.start_subslot();
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
                            }
                        }
                        let mut o0 = [Decoded::EMPTY; MAX_DECODED];
                        let mut o1 = [Decoded::EMPTY; MAX_DECODED];
                        let mut o2 = [Decoded::EMPTY; MAX_DECODED];
                        let n0 = rx0.finish_minute(&mut o0);
                        let n1 = rx1.finish_minute(&mut o1);
                        let n2 = rx2.finish_minute(&mut o2);
                        for (idx, (o, n)) in [(&o0, n0), (&o1, n1), (&o2, n2)].into_iter().enumerate() {
                            lu[idx] += o[..n]
                                .iter()
                                .filter(|d| d.frame != target && d.frame != neighbor)
                                .count();
                        }
                        loc.push([
                            o0[..n0].iter().any(|d| d.frame == target),
                            o1[..n1].iter().any(|d| d.frame == target),
                            o2[..n2].iter().any(|d| d.frame == target),
                        ]);
                    }
                    (loc, lu)
                }));
            }
            for h in handles {
                let (l, lu) = h.join().unwrap();
                res.extend(l);
                for k in 0..3 {
                    unexp[k] += lu[k];
                }
            }
        });
        for r in &res {
            for k in 0..3 {
                if r[k] {
                    hits[si][k] += 1;
                }
            }
        }
        outcomes.push(res);
    }

    let tvec = vec![trials; snrs.len()];
    let s50: [Option<f64>; 3] = core::array::from_fn(|k| {
        let hv: Vec<usize> = (0..snrs.len()).map(|s| hits[s][k]).collect();
        fit_logistic_50(snrs, &hv, &tvec)
    });
    let ref50 = s50[2].or(s50[0]).unwrap_or(-21.0);
    let knee = snrs
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| ((*a - ref50).abs()).total_cmp(&((*b - ref50).abs())))
        .unwrap()
        .0;

    let names = ["base_5bit", "four_bit_0", "hh_dual_4b"];
    println!("\n--- {name} ({trials} trials/SNR, knee={:+.1} dB) ---", snrs[knee]);
    for k in 0..3 {
        let x50 = s50[k].unwrap_or(f64::NAN);
        let diff = if let Some(b0) = s50[0] { x50 - b0 } else { f64::NAN };
        let (mut a_only, mut b_only) = (0, 0);
        for r in &outcomes[knee] {
            if r[0] && !r[k] { a_only += 1; }
            else if !r[0] && r[k] { b_only += 1; }
        }
        let p = if k == 0 { 1.0 } else { mcnemar_p(a_only, b_only) };
        println!(
            "  {:<11}: SNR50 = {:+6.2} dB (d={:+5.2} dB) | knee {:>3}/{:<3} (b={:>2}, c={:>2}, p={:.4}) | u={}",
            names[k], x50, diff, hits[knee][k], trials, a_only, b_only, p, unexp[k]
        );
    }
}

fn run_noise_far(minutes: usize) {
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    let chunk = minutes.div_ceil(n_threads);
    let mut fa = 0usize;
    thread::scope(|s| {
        let mut handles = Vec::new();
        for ti in 0..n_threads {
            let start = ti * chunk;
            let end = (start + chunk).min(minutes);
            if start >= end { continue; }
            handles.push(s.spawn(move || {
                let mut lfa = 0usize;
                let mut rx = RxDualPhaseHandheld::new(true, true);
                for i in start..end {
                    let noise_min = synth_noise_minute(2_000_000 + i as u64);
                    rx.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 { rx.start_subslot(); }
                        let base = sub * RATE as usize * 10;
                        for blk in noise_min[base..base + RATE as usize * 10].chunks(BLOCK) {
                            rx.push_samples(blk);
                        }
                    }
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    lfa += rx.finish_minute(&mut out);
                }
                lfa
            }));
        }
        for h in handles {
            fa += h.join().unwrap();
        }
    });
    println!("\n=== NOISE-ONLY FAR ({minutes} independent minutes) ===\n  hh_dual_4b: {fa}");
}

fn replay_wav(path: &str, start_sec: f64) {
    let mut f = File::open(path).expect("open wav");
    let mut buf = Vec::new();
    f.read_to_end(&mut buf).unwrap();
    let sr = u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]);
    // Find "data" chunk
    let mut pos = 12usize;
    let mut data_off = 44usize;
    let mut data_len = buf.len().saturating_sub(44);
    while pos + 8 <= buf.len() {
        let id = &buf[pos..pos + 4];
        let sz = u32::from_le_bytes([buf[pos + 4], buf[pos + 5], buf[pos + 6], buf[pos + 7]]) as usize;
        if id == b"data" {
            data_off = pos + 8;
            data_len = sz.min(buf.len() - data_off);
            break;
        }
        pos += 8 + sz;
    }
    let raw: Vec<i16> = buf[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    // Linearly resample sr -> 24,000 Hz if needed
    let pcm24: Vec<i16> = if sr == RATE {
        raw
    } else {
        let out_n = ((raw.len() as f64) * (RATE as f64) / (sr as f64)) as usize;
        (0..out_n)
            .map(|i| {
                let x = (i as f64) * (sr as f64) / (RATE as f64);
                let idx = (x as usize).min(raw.len().saturating_sub(2));
                let frac = x - idx as f64;
                ((raw[idx] as f64) * (1.0 - frac) + (raw[idx + 1] as f64) * frac).round() as i16
            })
            .collect()
    };
    let s0 = ((start_sec * RATE as f64) as usize).min(pcm24.len());
    let mut rx_base = CompactReceiver60::new();
    let mut rx_dual = RxDualPhaseHandheld::new(true, true);
    for sub in 0..REPEATS {
        if sub > 0 {
            rx_base.start_subslot();
            rx_dual.start_subslot();
        }
        let a = s0 + sub * RATE as usize * 10;
        let b = (a + RATE as usize * 10).min(pcm24.len());
        if a < b {
            for blk in pcm24[a..b].chunks(BLOCK) {
                rx_base.push_samples(blk);
                rx_dual.push_samples(blk);
            }
        }
    }
    let mut ob = [Decoded::EMPTY; MAX_DECODED];
    let mut od = [Decoded::EMPTY; MAX_DECODED];
    let nb = rx_base.finish_minute(&mut ob);
    let nd = rx_dual.finish_minute(&mut od);
    println!("WAV {path} (sr={sr}, start={start_sec:.2}s):");
    println!("  base_5bit  ({nb}): {:?}", &ob[..nb]);
    println!("  hh_dual_4b ({nd}): {:?}", &od[..nd]);
}

fn main() {
    let arg1 = std::env::args().nth(1).unwrap_or_default();
    if arg1 == "tones" {
        let en61 = encode_grid4(b"EN61").unwrap();
        for call in [b"NQ6N".as_slice(), b"WP2Z".as_slice()] {
            let f = Frame::Cq {
                sender: Callsign::encode(call).unwrap(),
                grid4: en61,
                tx_pwr: 11,
                reserved: 0,
            };
            let t: String = encode_tones(f).iter().map(|&x| (b'0' + x) as char).collect();
            println!("{}: {t}", core::str::from_utf8(call).unwrap());
        }
        return;
    }
    if arg1 == "replay" {
        let path = std::env::args().nth(2).expect("wav path");
        let start: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(0.0);
        replay_wav(&path, start);
        return;
    }

    let trials: usize = arg1.parse().unwrap_or(200);
    let noise_mins: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(1000);
    let iso_snrs = [-23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];
    let nbr_snrs = [-22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];

    println!("=== HANDHELD DUAL-PHASE (+40ms, 12,845 B) & 4-BIT PRIMARY CHECK ({trials} trials/SNR) ===");
    run_sweep("iso_2175_0ms", 2175, 400, None, &iso_snrs, trials, 100_000);
    run_sweep("iso_2175_+20ms", 2175, 420, None, &iso_snrs, trials, 110_000);
    run_sweep("iso_2175_-20ms", 2175, 380, None, &iso_snrs, trials, 120_000);
    run_sweep("iso_2175_+30ms", 2175, 430, None, &iso_snrs, trials, 130_000);
    run_sweep("iso_2175_+40ms", 2175, 440, None, &iso_snrs, trials, 140_000);
    run_sweep("iso_2175_-40ms", 2175, 360, None, &iso_snrs, trials, 150_000);
    run_sweep("nbr18_2175_0ms", 2175, 400, Some((1950, 440)), &nbr_snrs, trials, 160_000);
    run_sweep("nbr18_2175_+40ms", 2175, 440, Some((1950, 400)), &nbr_snrs, trials, 170_000);
    run_sweep("nbr18_2175_+5Hz_0ms", 2180, 400, Some((1950, 440)), &nbr_snrs, trials, 180_000);
    run_noise_far(noise_mins);
}
