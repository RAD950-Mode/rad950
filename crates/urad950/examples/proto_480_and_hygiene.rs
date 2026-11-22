//! Step 3 Prototype 1 & 2:
//!   1. Bin-aligned 480-point transform (12.5 Hz bins, exact 4-bin tones & 18-bin channels)
//!      in both 80 ms full-symbol and 70 ms zero-padded variants, evaluated at 0, ±5, ±6, ±10 Hz
//!      (isolated and +18 dB neighbor).
//!   2. Zero-RAM shortlist hygiene (adjacent-start suppression + MAX_CAND=24) on 1275 & 2175 Hz
//!      offset + strong-neighbor sweeps.
//!
//! Usage:
//!   cargo run --release --example proto_480_and_hygiene -- [trials_per_snr] [offset_trials]

use std::f64::consts::TAU;
use std::sync::OnceLock;
use std::thread;
use urad950::dsp::{log2_q8, lowpass, Decimator};
use urad950::ldpc::{self, BpScratch, N_BITS};
use urad950::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS, S0, S1, S2, SUBCHANNELS_HZ};
use urad950::rx::SYM_SAMPLES;
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

// 480-point bin-aligned parameters (df = 6000 / 480 = 12.5 Hz)
const N480: usize = 480;
const FIRST_BIN_480: usize = 46; // 575.0 Hz
const N_BINS_480: usize = 144; // 575.0 .. 2362.5 Hz (covers 600 - 25 Hz through 2325 + 37.5 Hz)
const COMPACT_CHANNELS: usize = SUBCHANNELS_HZ.len();
const COMPACT_FIRST_SYM: usize = 1 + data_sym_pos(0);
const COMPACT_LAST_SYM: usize = FOLD_SYMS - urad950::phy::N_SYMBOLS + data_sym_pos(N_DATA_SYMS - 1);
const COMPACT_SYMS: usize = COMPACT_LAST_SYM - COMPACT_FIRST_SYM + 1;

static LP_24K_TO_6K: [i16; 31] = lowpass::<31>(2_550.0 / 24_000.0);

/// Fast 480-point real DFT for bins `FIRST_BIN_480..FIRST_BIN_480 + N_BINS_480`
/// using 240-point complex FFT via mixed-radix `240 = 16 * 15` (exact, fast on host).
struct Dft480Tables {
    // Twiddles for 240-pt complex FFT: factor 240 = 16 * 15
    cos16: [[f64; 16]; 16],
    sin16: [[f64; 16]; 16],
    cos15: [[f64; 15]; 15],
    sin15: [[f64; 15]; 15],
    tw_r: [[f64; 16]; 15],
    tw_i: [[f64; 16]; 15],
    // Real-to-complex unpack twiddles for k = 1..240
    unp_c: [f64; 241],
    unp_s: [f64; 241],
}

static DFT480: OnceLock<Dft480Tables> = OnceLock::new();

fn dft480_tables() -> &'static Dft480Tables {
    DFT480.get_or_init(|| {
        let mut cos16 = [[0.0; 16]; 16];
        let mut sin16 = [[0.0; 16]; 16];
        for k in 0..16 {
            for n in 0..16 {
                let a = -TAU * (k * n) as f64 / 16.0;
                cos16[k][n] = a.cos();
                sin16[k][n] = a.sin();
            }
        }
        let mut cos15 = [[0.0; 15]; 15];
        let mut sin15 = [[0.0; 15]; 15];
        for k in 0..15 {
            for n in 0..15 {
                let a = -TAU * (k * n) as f64 / 15.0;
                cos15[k][n] = a.cos();
                sin15[k][n] = a.sin();
            }
        }
        let mut tw_r = [[0.0; 16]; 15];
        let mut tw_i = [[0.0; 16]; 15];
        for n1 in 0..15 {
            for k2 in 0..16 {
                let a = -TAU * (n1 * k2) as f64 / 240.0;
                tw_r[n1][k2] = a.cos();
                tw_i[n1][k2] = a.sin();
            }
        }
        let mut unp_c = [0.0; 241];
        let mut unp_s = [0.0; 241];
        for k in 0..=240 {
            let a = -TAU * k as f64 / 480.0;
            unp_c[k] = a.cos();
            unp_s[k] = a.sin();
        }
        Dft480Tables {
            cos16,
            sin16,
            cos15,
            sin15,
            tw_r,
            tw_i,
            unp_c,
            unp_s,
        }
    })
}

/// Compute exact 480-point real FFT power `|X[k]|^2` for `k in FIRST_BIN_480..FIRST_BIN_480+N_BINS_480`
/// using a 240-point complex FFT (`16 x 15` Cooley-Tukey) + real-FFT unpack.
fn rfft480_power(x: &[i16; N480], out_pwr: &mut [f64; N_BINS_480]) {
    let t = dft480_tables();
    // Pack 480 real samples into 240 complex samples: z[n] = x[2n] + j*x[2n+1]
    // Index as n = 15*n2 + n1 (n2 in 0..16, n1 in 0..15)
    let mut tmp_r = [[0.0f64; 16]; 15];
    let mut tmp_i = [[0.0f64; 16]; 15];

    // Step 1: 15 DFTs of length 16 + twiddle multiply
    for n1 in 0..15 {
        let mut xr = [0.0f64; 16];
        let mut xi = [0.0f64; 16];
        for n2 in 0..16 {
            let idx = 15 * n2 + n1;
            xr[n2] = x[2 * idx] as f64;
            xi[n2] = x[2 * idx + 1] as f64;
        }
        for k2 in 0..16 {
            let mut sr = 0.0;
            let mut si = 0.0;
            let c = &t.cos16[k2];
            let s = &t.sin16[k2];
            for n2 in 0..16 {
                sr += xr[n2] * c[n2] - xi[n2] * s[n2];
                si += xr[n2] * s[n2] + xi[n2] * c[n2];
            }
            let wr = t.tw_r[n1][k2];
            let wi = t.tw_i[n1][k2];
            tmp_r[n1][k2] = sr * wr - si * wi;
            tmp_i[n1][k2] = sr * wi + si * wr;
        }
    }

    // Step 2: 16 DFTs of length 15 -> Z[k = 16*k1 + k2]
    let mut zr = [0.0f64; 240];
    let mut zi = [0.0f64; 240];
    for k2 in 0..16 {
        for k1 in 0..15 {
            let mut sr = 0.0;
            let mut si = 0.0;
            let c = &t.cos15[k1];
            let s = &t.sin15[k1];
            for n1 in 0..15 {
                let ar = tmp_r[n1][k2];
                let ai = tmp_i[n1][k2];
                sr += ar * c[n1] - ai * s[n1];
                si += ar * s[n1] + ai * c[n1];
            }
            let k = 16 * k1 + k2;
            zr[k] = sr;
            zi[k] = si;
        }
    }

    // Step 3: Unpack real FFT bins k in FIRST_BIN_480..FIRST_BIN_480 + N_BINS_480
    for b in 0..N_BINS_480 {
        let k = FIRST_BIN_480 + b;
        let mk = 240 - k;
        let zk_r = zr[k];
        let zk_i = zi[k];
        let zmk_r = zr[mk];
        let zmk_i = zi[mk];
        // Even part E[k] = 0.5*(Z[k] + conj(Z[240-k]))
        let er = 0.5 * (zk_r + zmk_r);
        let ei = 0.5 * (zk_i - zmk_i);
        // Odd part O[k] = -0.5j*(Z[k] - conj(Z[240-k])) = 0.5*((zk_i + zmk_i) - j*(zk_r - zmk_r))
        let or_ = 0.5 * (zk_i + zmk_i);
        let oi = -0.5 * (zk_r - zmk_r);
        let wr = t.unp_c[k];
        let wi = t.unp_s[k];
        let xr = er + (or_ * wr - oi * wi);
        let xi = ei + (or_ * wi + oi * wr);
        out_pwr[b] = xr * xr + xi * xi;
    }
}

/// Prototype 480-point bin-aligned RAD950-60 receiver (same 5-bit fold + 1,236-byte compact table).
struct Rx480 {
    full_80ms: bool,
    hygiene_max_cand: usize,
    hygiene_adj_sym: bool,
    decim: Decimator<31>,
    pcm_buf: [i16; SYM_SAMPLES],
    pcm_pos: usize,
    decim_count: u32,
    fold: [[u8; N_BINS_480]; FOLD_SYMS],
    compact: [[[u8; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
}

impl Rx480 {
    fn new(full_80ms: bool, hygiene_max_cand: usize, hygiene_adj_sym: bool) -> Self {
        Self {
            full_80ms,
            hygiene_max_cand,
            hygiene_adj_sym,
            decim: Decimator::new(),
            pcm_buf: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            fold: [[0; N_BINS_480]; FOLD_SYMS],
            compact: [[[0; 4]; COMPACT_SYMS]; COMPACT_CHANNELS],
        }
    }

    fn reset(&mut self) {
        self.decim.reset();
        self.pcm_buf.fill(0);
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
        self.pcm_buf.fill(0);
        self.pcm_pos = 0;
        self.decim_count = 0;
    }

    fn push_samples(&mut self, samples: &[i16]) {
        for &s in samples {
            if let Some(y) = self.decim.push(s, &LP_24K_TO_6K, 4) {
                self.pcm_buf[self.pcm_pos] = y;
                self.pcm_pos += 1;
                self.decim_count = self.decim_count.wrapping_add(1);
                if self.pcm_pos == SYM_SAMPLES {
                    self.pcm_pos = 0;
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
        let mut x = [0i16; N480];
        if self.full_80ms {
            x.copy_from_slice(&self.pcm_buf);
        } else {
            x[..SYM_SAMPLES - RAMP_SAMPLES_6K].copy_from_slice(&self.pcm_buf[RAMP_SAMPLES_6K..]);
        }
        let mut pwr = [0.0f64; N_BINS_480];
        rfft480_power(&x, &mut pwr);

        let mut cur_bins = [0u8; N_BINS_480];
        let mut sum = 0u32;
        let mut peak = 0u8;
        for b in 0..N_BINS_480 {
            // Same log2_q8/32 scale as rx60.rs
            let p_u64 = (pwr[b].max(1.0)) as u64;
            let lp = (log2_q8(p_u64) - (13 << 8)) >> 5;
            let v = lp.clamp(0, 255) as u8;
            cur_bins[b] = v;
            sum += v as u32;
            if v > peak {
                peak = v;
            }
        }
        let floor = (sum / N_BINS_480 as u32) as i32;
        let span = (i32::from(peak) - floor).clamp(18, 96);
        for b in 0..N_BINS_480 {
            let q = Self::quantize(i32::from(cur_bins[b]), floor, span) as u8;
            self.fold[sym][b] = self.fold[sym][b].saturating_add(q).min(31);
        }

        // Accumulate compact nominal-channel corrections (exact integer bins!)
        if sym >= COMPACT_FIRST_SYM && sym <= COMPACT_LAST_SYM {
            let mut counts = [0u8; 32];
            for &v in &cur_bins {
                counts[usize::from(v >> 3)] += 1;
            }
            let mut above = 0usize;
            let mut upper = 0i32;
            for (bucket, &count) in counts.iter().enumerate().rev() {
                above += usize::from(count);
                if above > N_BINS_480 / 20 {
                    upper = (bucket * 8 + 7) as i32;
                    break;
                }
            }
            let pct_span = (upper - floor).clamp(18, 96);
            if pct_span < span {
                let c_sym = sym - COMPACT_FIRST_SYM;
                for (ch, &ch_hz) in SUBCHANNELS_HZ.iter().enumerate() {
                    let b0 = (ch_hz as usize * 2 / 25) - FIRST_BIN_480; // exact integer bin!
                    for tone in 0..4 {
                        let lp = i32::from(cur_bins[b0 + tone * 4]);
                        let base_q = Self::quantize(lp, floor, span);
                        let pct_q = Self::quantize(lp, floor, pct_span);
                        let boost = (pct_q - base_q).max(0) as u8;
                        if boost > 0 {
                            self.compact[ch][c_sym][tone] =
                                self.compact[ch][c_sym][tone].saturating_add(boost).min(7);
                        }
                    }
                }
            }
        }
    }

    #[inline]
    fn sample_tone_half(&self, sym: usize, f0_half: usize, tone: usize) -> i32 {
        // Tone spacing is EXACTLY 4 bins = 8 half-bins!
        let pos_half = f0_half + tone * 8;
        let b = pos_half >> 1;
        if b + 1 >= N_BINS_480 || sym >= FOLD_SYMS {
            return 0;
        }
        let v0 = i32::from(self.fold[sym][b]);
        if (pos_half & 1) == 0 {
            v0 * 16
        } else {
            let v1 = i32::from(self.fold[sym][b + 1]);
            (v0 + v1) * 8
        }
    }

    fn decode_primary(&self, start_sym: u8, f0_half: u16, scratch: &mut BpScratch) -> Option<Frame> {
        let mut inter_llr = [0i32; N_BITS];
        for d in 0..N_DATA_SYMS {
            let s = start_sym as usize + data_sym_pos(d);
            let m0 = self.sample_tone_half(s, f0_half as usize, 0);
            let m1 = self.sample_tone_half(s, f0_half as usize, 1);
            let m2 = self.sample_tone_half(s, f0_half as usize, 2);
            let m3 = self.sample_tone_half(s, f0_half as usize, 3);
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
        let b0 = (SUBCHANNELS_HZ[ch] as usize * 2 / 25) - FIRST_BIN_480;
        let f0_half = b0 * 2;
        let mut interleaved = [0i8; N_BITS];
        for d in 0..N_DATA_SYMS {
            let s = usize::from(start_sym) + data_sym_pos(d);
            let c_sym = s - COMPACT_FIRST_SYM;
            let tones: [i32; 4] = core::array::from_fn(|t| {
                self.sample_tone_half(s, f0_half, t) + i32::from(self.compact[ch][c_sym][t]) * 16
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
        let max_cand = self.hygiene_max_cand;
        let mut best: Vec<(i32, u8, u16)> = vec![(i32::MIN, 0, 0); max_cand];
        let mut n_best = 0usize;
        let max_f0_half = (N_BINS_480 - 2) * 2 - 24; // 3 * 8 = 24 half-bins
        let mut compact_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];
        let nom_half: [usize; COMPACT_CHANNELS] =
            core::array::from_fn(|ch| ((SUBCHANNELS_HZ[ch] as usize * 2 / 25) - FIRST_BIN_480) * 2);

        for start_sym in 1usize..=(FOLD_SYMS - 110) {
            let mut nom_ch = 0usize;
            for f0_half in 0..=max_f0_half {
                let mut score = 0i32;
                for (k, &t) in S0.iter().enumerate() {
                    let s = start_sym + k;
                    let sig = self.sample_tone_half(s, f0_half, t as usize);
                    let bg = (self.sample_tone_half(s, f0_half, 0)
                        + self.sample_tone_half(s, f0_half, 1)
                        + self.sample_tone_half(s, f0_half, 2)
                        + self.sample_tone_half(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S1.iter().enumerate() {
                    let s = start_sym + 18 + k;
                    let sig = self.sample_tone_half(s, f0_half, t as usize);
                    let bg = (self.sample_tone_half(s, f0_half, 0)
                        + self.sample_tone_half(s, f0_half, 1)
                        + self.sample_tone_half(s, f0_half, 2)
                        + self.sample_tone_half(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }
                for (k, &t) in S2.iter().enumerate() {
                    let s = start_sym + 104 + k;
                    let sig = self.sample_tone_half(s, f0_half, t as usize);
                    let bg = (self.sample_tone_half(s, f0_half, 0)
                        + self.sample_tone_half(s, f0_half, 1)
                        + self.sample_tone_half(s, f0_half, 2)
                        + self.sample_tone_half(s, f0_half, 3))
                        >> 2;
                    score += sig - bg;
                }

                if nom_ch < COMPACT_CHANNELS && f0_half == nom_half[nom_ch] {
                    let bs = &mut compact_sync[nom_ch];
                    if score > bs[0].0 {
                        bs[1] = bs[0];
                        bs[0] = (score, start_sym as u8);
                    } else if score > bs[1].0 {
                        bs[1] = (score, start_sym as u8);
                    }
                    nom_ch += 1;
                }

                if n_best == max_cand && score <= best[max_cand - 1].0 {
                    continue;
                }
                let mut dup_idx = None;
                for (i, item) in best.iter().enumerate().take(n_best) {
                    let sym_match = if self.hygiene_adj_sym {
                        item.1.abs_diff(start_sym as u8) <= 1
                    } else {
                        item.1 == start_sym as u8
                    };
                    let df_tol = if self.hygiene_adj_sym { 4 } else { 3 };
                    if sym_match && (item.2 as usize).abs_diff(f0_half) <= df_tol {
                        dup_idx = Some(i);
                        break;
                    }
                }
                if let Some(di) = dup_idx {
                    if score > best[di].0 {
                        best[di] = (score, start_sym as u8, f0_half as u16);
                    }
                } else if n_best < max_cand {
                    best[n_best] = (score, start_sym as u8, f0_half as u16);
                    n_best += 1;
                } else {
                    best[max_cand - 1] = (score, start_sym as u8, f0_half as u16);
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
                let audio_hz = ((FIRST_BIN_480 * 2 + f0_half as usize) * 25 / 4) as u16;
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

    // 4 receivers: [0] 512pt baseline compact, [1] 480pt 80ms compact (std cand),
    //              [2] 480pt 70ms compact (std cand), [3] 480pt 80ms + hygiene_24
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
                    let mut rx1 = Rx480::new(true, 10, false);
                    let mut rx2 = Rx480::new(false, 10, false);
                    let mut rx3 = Rx480::new(true, 24, true);

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

    let names = ["512_base", "480_80ms", "480_70ms", "480_80+h24"];
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
            "  {:<11}: SNR50 = {:+6.2} dB (d={:+5.2} dB) | knee {:>3}/{:<3} (base_only={:>2}, cand_only={:>2}, p={:.4}) | unexp={}",
            names[k], x50, diff, hits[knee][k], trials, a_only, b_only, p, unexp[k]
        );
    }
}

fn main() {
    let trials: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(160);
    let iso_snrs = [-24.0, -23.5, -23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0];
    let nbr_snrs = [-23.5, -23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5];

    println!("=== 480-pt BIN-ALIGNED TRANSFORM + SHORTLIST HYGIENE ({trials} trials/SNR) ===");
    run_sweep("iso_1275_0Hz", 1275, None, &iso_snrs, trials, 500_000);
    run_sweep("iso_1275_+5Hz", 1280, None, &iso_snrs, trials, 510_000);
    run_sweep("iso_1275_+6Hz_midbin", 1281, None, &iso_snrs, trials, 520_000);
    run_sweep("iso_1275_+10Hz", 1285, None, &iso_snrs, trials, 530_000);
    run_sweep("iso_2175_0Hz", 2175, None, &iso_snrs, trials, 540_000);
    run_sweep("nbr18_2175_0Hz", 2175, Some(1950), &nbr_snrs, trials, 550_000);
    run_sweep("nbr18_1275_0Hz", 1275, Some(1050), &nbr_snrs, trials, 560_000);
    run_sweep("nbr18_1275_+5Hz", 1280, Some(1050), &nbr_snrs, trials, 570_000);
    run_sweep("nbr18_1275_+10Hz", 1285, Some(1050), &nbr_snrs, trials, 580_000);
    run_sweep("nbr18_2175_+5Hz", 2180, Some(1950), &nbr_snrs, trials, 590_000);
    run_sweep("nbr18_2175_+10Hz", 2185, Some(1950), &nbr_snrs, trials, 600_000);
}
