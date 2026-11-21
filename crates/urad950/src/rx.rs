//! RAD950 streaming receiver fitting strictly inside `ferrite_core::decoders::BUDGET` (`13,312` bytes),
//! including internal LDPC belief-propagation scratch (`work` shared with the early-acquisition ring buffer).
//!
//! Pipeline:
//! 1. `24,000 Hz -> 6,000 Hz` 4:1 low-pass FIR decimator (`udsp::fir::Decimator<31>`).
//! 2. Sliding 480-sample (`80 ms`) Hann-windowed 512-point real FFT (`udsp::fft`) evaluated every `240` samples
//!    (`40 ms` half-symbol phase `0` and `1`), with block-exponent (`1 << shift`) log-power normalization.
//! 3. `32-symbol x 2-phase x 150-bin` packed ring buffer (`9,600 B` in `work`) holding `586..2,344 Hz` across all
//!    arrival skews `start_sym in 1..=9` (`[-320 ms, +360 ms]` relative to nominal `+400 ms` start).
//! 4. Clamped contrast `S0 + S1` 2D acquisition at slot symbol `32` (`t = 2.64 s`) admitting `K = 6` tracks with
//!    frequency non-maximum suppression and separated-peak fairness reservation, backfilling `data[0..12]` and
//!    streaming `data[12..92]` across 3 linear drift branches (`-11.7 Hz, 0 Hz, +11.7 Hz` over the burst) selected
//!    at end-of-burst by `S2` (`104..110`) tail-sync scores.
//! 5. Serial normalized min-sum `(184, 78)` LDPC + CRC-14 decoding at end of slot reusing `work` for BP scratch.

use crate::frame::Frame;
use crate::ldpc::{self, N_BITS, N_EDGES};
use crate::phy::{interleave_idx, DEFAULT_SPACING_HZ, N_DATA_SYMS, N_SYMBOLS, S0, S1, S2};
use udsp::fft::{cfft, real_power};
use udsp::fir::Decimator;
use udsp::log::log2_q8;
use udsp::mix::lowpass;
use udsp::trig::ccos;

pub const INPUT_RATE_HZ: u32 = 24_000;
pub const DECIM_FACTOR: usize = 4;
pub const PROC_RATE_HZ: u32 = INPUT_RATE_HZ / DECIM_FACTOR as u32; // 6,000 Hz
pub const SYM_SAMPLES: usize = 480; // 80 ms @ 6,000 Hz
pub const HALF_SAMPLES: usize = 240; // 40 ms @ 6,000 Hz
pub const FFT_N: usize = 512;
pub const FFT_M: usize = FFT_N / 2; // 256 complex points

/// First FFT bin stored (`50 * 6000 / 512 = 585.9 Hz`) and number of bins (`50..200`, up to `2,332.0 Hz`).
pub const FIRST_BIN: usize = 50;
pub const N_BINS: usize = 150;

/// Ring depth in whole symbols (`1..=32` covers `start_sym in 1..=9` through `S1` end `9 + 23 = 32`).
pub const RING_SYMS: usize = 32;
pub const MIN_START_SYM: usize = 1; // +80 ms (-320 ms vs +400 ms nominal)
pub const NOMINAL_START_SYM: usize = 5; // +400 ms nominal
pub const MAX_START_SYM: usize = 9; // +720 ms (+320 ms vs +400 ms nominal; +360 ms on phase 1)
pub const ACQUIRE_AT_SYM: usize = MAX_START_SYM + 23; // 32

pub const MAX_TRACKS: usize = 8;
pub const MAX_DECODED: usize = 8;
pub const LDPC_MAX_ITER: usize = 35;

/// Data-symbol index (`36..92`, i.e. `rel_sym = 48..104`) from which negative/positive linear drift branches diverge:
/// - `rel_sym in 48..83` (`d in 36..71`, 35 symbols): `+/- 1` half-bin (`+/- 5.86 Hz`) intermediate correction
/// - `rel_sym in 83..104` (`d in 71..92`, 21 symbols): `+/- 2` half-bins (`+/- 11.72 Hz`) end-of-burst correction
pub const DRIFT_START_DATA: usize = 36;
const TAIL_DATA_SYMS: usize = N_DATA_SYMS - DRIFT_START_DATA; // 56 dibits (packed 1 byte per dibit = 56 bytes)

/// Half-bin drift magnitude (`0`, `1`, or `2`) applied along the negative/positive linear drift trajectories at
/// burst-relative symbol `rel_sym in 0..110` (nearest-integer rounding of a `0 -> 2` half-bin ramp from `S0/S1`
/// center `rel_sym = 12` to `S2` center `rel_sym = 107`).
#[inline]
pub const fn drift_step_half_bins(rel_sym: usize) -> u16 {
    if rel_sym < 48 {
        0
    } else if rel_sym < 83 {
        1
    } else {
        2
    }
}

#[inline]
fn pack_dibit_i4(b0: i8, b1: i8) -> u8 {
    // Scale `[-24, +24]` by `/ 3` into signed 4-bit `[-8, +7]` nibbles.
    let q0 = ((b0 as i16 + if b0 >= 0 { 1 } else { -1 }) / 3).clamp(-8, 7) as i8;
    let q1 = ((b1 as i16 + if b1 >= 0 { 1 } else { -1 }) / 3).clamp(-8, 7) as i8;
    (((q0 as u8) & 0x0F) << 4) | ((q1 as u8) & 0x0F)
}

#[inline]
fn unpack_dibit_i4(packed: u8) -> (i8, i8) {
    // Sign-extend 4-bit nibbles and restore `* 3` LLR scale (`[-24, +21]`).
    let q0 = ((packed as i8) >> 4) * 3;
    let q1 = (((packed << 4) as i8) >> 4) * 3;
    (q0, q1)
}

/// Packed `i16` length holding `RING_SYMS * 2 * N_BINS = 9,600` bytes during capture, and reused after
/// `ACQUIRE_AT_SYM` (`sym == 32`) for:
/// - `0..840` (`HIST_LEN`): `140 ms` circular `6,000 Hz` PCM ring for rectangular `dt`-aligned Goertzel demodulation.
/// - `840..3400` (`8 tracks x 80 streaming data symbols x 4 tones`): quantized `log2_q8` rectangular tone energies.
/// - `3400..3464` (`8 tracks x 8 i16`): `S2` rectangular `on_sum`/`off_sum` (`f32` bit-packed into `i16` pairs).
/// - `3528..4237`: `post: [i16; 184]` + `c2v: [i16; 525]` LDPC belief-propagation scratch in `finish_slot()`.
const WORK_BYTES: usize = RING_SYMS * 2 * N_BINS; // 9,600
const WORK_I16: usize = WORK_BYTES / 2; // 4,800
const HIST_LEN: usize = 840; // 140 ms @ 6,000 Hz (covers +-20 ms timing shift around 80 ms symbol)
const REF_START_DATA: usize = 12; // data[12..92] = 80 streaming symbols
const REF_DATA_SYMS: usize = N_DATA_SYMS - REF_START_DATA; // 80
const REF_STRIDE: usize = REF_DATA_SYMS * 4; // 320 i16 per track
const REF_BASE: usize = HIST_LEN; // 840..3400 (2,560 i16)
const S2_BASE: usize = REF_BASE + MAX_TRACKS * REF_STRIDE; // 3400..3432 (32 i16 = 8 * 2 f32)
const BP_BASE: usize = 3528; // 3528..4237 (709 i16 < 4800)

/// Fast `no_std` IEEE-754 single-precision square root (accurate to `< 1e-6`).
#[inline]
fn sqrt_f32(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut y = f32::from_bits((x.to_bits() >> 1) + 0x1fc0_0000);
    y = 0.5 * (y + x / y);
    y = 0.5 * (y + x / y);
    0.5 * (y + x / y)
}

/// Fast `no_std` `2^x` for `x in [-60, 60]` (accurate to `< 2e-4`).
#[inline]
fn exp2_f32(x: f32) -> f32 {
    let xc = x.clamp(-60.0, 60.0);
    let n = if xc >= 0.0 { xc as i32 } else { (xc as i32) - 1 };
    let f = xc - n as f32;
    let p = 1.0 + f * (core::f32::consts::LN_2 + f * (0.240_226_5 + f * (0.055_504_1 + f * 0.011_122_2)));
    let scale = f32::from_bits(((n + 127) as u32) << 23);
    p * scale
}

/// Fast `no_std` natural logarithm `ln(x)` for `x > 0` (accurate to `< 1e-5`).
#[inline]
fn ln_f32(x: f32) -> f32 {
    if x <= 0.0 {
        return -80.0;
    }
    let bits = x.to_bits();
    let e = ((bits >> 23) & 0xFF) as i32 - 127;
    let m = f32::from_bits((bits & 0x007F_FFFF) | 0x3F80_0000);
    let u = (m - 1.0) / (m + 1.0);
    let u2 = u * u;
    let ln_m = 2.0 * u * (1.0 + u2 * (0.333_333_34 + u2 * (0.2 + u2 * 0.142_857_15)));
    (e as f32) * core::f32::consts::LN_2 + ln_m
}

/// `ln(I_0(x))` for `x >= 0` (Abramowitz & Stegun 9.8.1 / 9.8.2), exact to `< 1e-5`.
#[inline(never)]
fn ln_bessel_i0_f32(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    if x < 3.75 {
        let t = (x / 3.75) * (x / 3.75);
        let i0 = 1.0
            + t * (3.515_622_9
                + t * (3.089_942_4
                    + t * (1.206_749_2 + t * (0.265_973_2 + t * (0.036_076_8 + t * 0.004_581_3)))));
        ln_f32(i0)
    } else {
        let t = 3.75 / x;
        let p = 0.398_942_3
            + t * (0.013_285_92
                + t * (0.002_253_19
                    + t * (-0.001_575_65
                        + t * (0.009_162_81
                            + t * (-0.020_577_06
                                + t * (0.026_355_37 + t * (-0.016_476_33 + t * 0.003_923_77)))))));
        x + ln_f32((p / sqrt_f32(x)).max(1e-30))
    }
}

#[inline(never)]
fn log_sum_exp_f32(a: f32, b: f32) -> f32 {
    let m = if a > b { a } else { b };
    let d = if a > b { a - b } else { b - a };
    if d > 12.0 {
        m
    } else {
        m + ln_f32(1.0 + exp2_f32(-d * core::f32::consts::LOG2_E))
    }
}

/// Pure-`f32` `cos(w)` for `w in [0, pi]` (zero `f64` soft-float code on Cortex-M4F, accurate to `< 2e-6`).
#[inline(never)]
fn cos_f32(w: f32) -> f32 {
    let (x, sign) = if w > core::f32::consts::FRAC_PI_2 {
        (core::f32::consts::PI - w, -1.0f32)
    } else {
        (w, 1.0f32)
    };
    let x2 = x * x;
    sign * (1.0 + x2 * (-0.5 + x2 * (0.041_666_668 + x2 * (-0.001_388_889 + x2 * 2.480_158_8e-5))))
}

/// Rectangular 480-sample (`80 ms`) Goertzel tone energy over circular `hist` (`HIST_LEN = 840`).
#[inline(never)]
fn goertzel_energy(hist: &[i16], start_idx: usize, freq_hz: f32) -> f32 {
    let w = core::f32::consts::TAU * freq_hz / (PROC_RATE_HZ as f32);
    let c2 = 2.0 * cos_f32(w);
    let mut s1 = 0.0f32;
    let mut s2 = 0.0f32;
    let mut idx = start_idx;
    for _ in 0..SYM_SAMPLES {
        let s0 = hist[idx] as f32 + c2 * s1 - s2;
        s2 = s1;
        s1 = s0;
        idx += 1;
        if idx == HIST_LEN {
            idx = 0;
        }
    }
    let e = s1 * s1 + s2 * s2 - c2 * s1 * s2;
    if e > 1.0 { e } else { 1.0 }
}

/// Pack positive `f32` energy into `u16` Q8 `log2(energy)` (`0.0117 dB` resolution over `0..768 dB`).
#[inline]
fn pack_energy_q8(e: f32) -> u16 {
    let bits = e.max(1.0).to_bits();
    let exp = (((bits >> 23) & 0xFF) as i32 - 127).max(0) as u32;
    let m = f32::from_bits((bits & 0x007F_FFFF) | 0x3F80_0000);
    // `log2(m)` for `m in [1, 2)` via linear+quadratic fit: accurate to `< 0.002` octaves (`0.006 dB`).
    let f = m - 1.0;
    let frac = f * (core::f32::consts::LOG2_E - f * (0.491_15 - f * 0.048_45));
    let q8 = (exp << 8) + (frac * 256.0 + 0.5) as u32;
    q8.min(u16::MAX as u32) as u16
}

#[inline]
fn write_f32_work(work: &mut [i16], idx: usize, v: f32) {
    let b = v.to_bits();
    work[idx] = (b & 0xFFFF) as i16;
    work[idx + 1] = ((b >> 16) & 0xFFFF) as i16;
}

#[inline]
fn read_f32_work(work: &[i16], idx: usize) -> f32 {
    let lo = work[idx] as u16 as u32;
    let hi = work[idx + 1] as u16 as u32;
    f32::from_bits(lo | (hi << 16))
}

static DECIM_TAPS: [i16; 31] = lowpass::<31>(2500.0 / 24000.0);
static HANN_480: [i16; SYM_SAMPLES] = hann_table();
/// `2^(k/8) * 256` for `k = 0..8`, used to reconstruct unbiased linear power from `1/8`-octave log bins.
static EXP8: [u32; 8] = [256, 279, 304, 332, 362, 395, 431, 470];

const fn hann_table() -> [i16; SYM_SAMPLES] {
    let mut w = [0i16; SYM_SAMPLES];
    let mut n = 0;
    while n < SYM_SAMPLES {
        let v = 0.5 * (1.0 - ccos(2.0 * core::f64::consts::PI * n as f64 / SYM_SAMPLES as f64));
        let q = (v * 32767.0 + 0.5) as i32;
        w[n] = if q > 32767 { 32767 } else { q as i16 };
        n += 1;
    }
    w
}

/// Reconstruct scaled linear power (`u64`) from a quantized `1/8`-octave log-power byte `lp`.
#[inline]
fn lin_power(lp: u8) -> u64 {
    let oct = (lp >> 3).min(31) as u32;
    let frac = (lp & 7) as usize;
    (EXP8[frac] as u64) << oct
}

/// Decoded frame together with its measured physical-layer metadata.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decoded {
    pub frame: Frame,
    /// Estimated lowest-tone audio frequency offset in Hz above USB dial (at start of burst).
    pub audio_hz: u16,
    /// Arrival offset in milliseconds relative to nominal `+400 ms` start (`[-320, +360]`).
    pub time_offset_ms: i16,
    /// Estimated SNR in dB referenced to a `2,500 Hz` noise bandwidth (`[-24, +20]`).
    pub snr_db: i8,
}

/// Cooperative-RX soft candidate observation (`92`-byte signed 4-bit `[-7..=+7]` LDPC codeword LLR report,
/// `0` = no evidence, canonical LDPC bit order per `docs/rad950-cooperative-reception.md`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftCandidate {
    /// Estimated lowest-tone audio frequency offset in Hz above USB dial (refined `df`).
    pub audio_hz: u16,
    /// Arrival offset in milliseconds relative to nominal `+400 ms` start (`[-320, +360]`).
    pub time_offset_ms: i16,
    /// Estimated local SNR in dB (`2,500 Hz` reference).
    pub snr_db: i8,
    /// `S2` tail-sync score for candidate ranking / association confidence.
    pub sync_score: i16,
    /// 184 signed 4-bit LLRs (`[-7..=+7]`, two's complement nibbles packed 2 per byte: low nibble = even bit,
    /// high nibble = odd bit) in canonical LDPC codeword order.
    pub llrs_4bit: [u8; N_BITS / 2],
}

impl SoftCandidate {
    pub const EMPTY: Self = Self {
        audio_hz: 0,
        time_offset_ms: 0,
        snr_db: -24,
        sync_score: 0,
        llrs_4bit: [0; N_BITS / 2],
    };

    /// Quantize canonical LDPC codeword LLRs (`[-120..=+120]`) into packed signed 4-bit `[-7..=+7]` nibbles.
    pub fn pack_llrs(deinter: &[i8; N_BITS]) -> [u8; N_BITS / 2] {
        let mut out = [0u8; N_BITS / 2];
        let mut max_abs = 1i32;
        for &v in deinter {
            let a = (v as i32).abs();
            if a > max_abs {
                max_abs = a;
            }
        }
        let denom = max_abs.max(24);
        for i in 0..(N_BITS / 2) {
            let q = |v: i8| -> u8 {
                let num = (v as i32) * 7;
                let rounded = if num >= 0 { (num + denom / 2) / denom } else { (num - denom / 2) / denom };
                (rounded.clamp(-7, 7) as i8 as u8) & 0x0F
            };
            out[i] = q(deinter[2 * i]) | (q(deinter[2 * i + 1]) << 4);
        }
        out
    }
}

impl Decoded {
    /// Placeholder slot value for initializing `[Decoded; MAX_DECODED]` output buffers.
    pub const EMPTY: Self = Self {
        frame: Frame::Cq {
            sender: crate::frame::Callsign::from_raw(0).unwrap(),
            grid4: 0,
            tx_pwr: 0,
            reserved: 0,
        },
        audio_hz: 0,
        time_offset_ms: 0,
        snr_db: 0,
    };
}

#[derive(Clone, Copy)]
struct Track {
    active: bool,
    start_sym: u8,
    phase: u8,
    /// Fine timing offset in units of 4 decimated samples (`0.667 ms`, range `[-30..+30]` = `+-20 ms`).
    dt_q4: i8,
    q0: u16, // half-bin index relative to FIRST_BIN
    /// Fine frequency offset relative to `q0` in units of `0.25 Hz` (range `[-24..+24]` = `+-6.0 Hz`).
    df_q2: i8,
    /// First data symbol index (`>= REF_START_DATA`) covered by streaming rectangular Goertzel in `work`.
    ref_first_d: u8,
    /// Sum of linear power on the 18 active sync tones (`S0 + S1 + S2`, evaluated along winning drift).
    on_pwr_sum: u64,
    /// Sum of linear power across the `18 * 3 = 54` inactive sync tones (`3 x` noise per bin).
    off_pwr_sum: u64,
    /// `S2` (`104..110`) contrast scores for end-of-burst drift offsets `[-2, 0, +2]` half-bins (`-11.7, 0, +11.7 Hz`).
    s2_scores: [i16; 3],
    /// Zero-drift LLRs across all 92 data symbols (`184` bits).
    llr_zero: [i8; N_BITS],
    /// Packed 4-bit dibit LLRs for `d = 36..92` along negative linear drift (`-1` then `-2` half-bins).
    llr_neg_tail: [u8; TAIL_DATA_SYMS],
    /// Packed 4-bit dibit LLRs for `d = 36..92` along positive linear drift (`+1` then `+2` half-bins).
    llr_pos_tail: [u8; TAIL_DATA_SYMS],
}

impl Track {
    const fn empty() -> Self {
        Self {
            active: false,
            start_sym: 0,
            phase: 0,
            dt_q4: 0,
            q0: 0,
            df_q2: 0,
            ref_first_d: N_DATA_SYMS as u8,
            on_pwr_sum: 0,
            off_pwr_sum: 0,
            s2_scores: [0; 3],
            llr_zero: [0; N_BITS],
            llr_neg_tail: [0; TAIL_DATA_SYMS],
            llr_pos_tail: [0; TAIL_DATA_SYMS],
        }
    }

    #[inline]
    fn base_freq_hz(&self) -> f32 {
        let q0_hz = (2 * FIRST_BIN as u32 + self.q0 as u32) as f32 * (PROC_RATE_HZ as f32 / (2.0 * FFT_N as f32));
        q0_hz + (self.df_q2 as f32) * 0.25
    }
}

/// Streaming RAD950 slot receiver (`<= 13,312` bytes including internal LDPC scratch, zero heap allocation).
pub struct Receiver {
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: u16,
    acquired: bool,
    wf_ready: bool,
    decim_count: u32,
    tone_q_off: [u16; 4],
    wf_col: [u8; 12],
    work: [i16; WORK_I16],
    tracks: [Track; MAX_TRACKS],
}

impl Receiver {
    pub const fn new() -> Self {
        Self::with_spacing(DEFAULT_SPACING_HZ)
    }

    pub const fn with_spacing(spacing_hz: u32) -> Self {
        let tone_q_off = compute_tone_q_off(spacing_hz);
        Self {
            decim: Decimator::new(),
            pcm_ring: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            tone_q_off,
            work: [0; WORK_I16],
            tracks: [Track::empty(); MAX_TRACKS],
            acquired: false,
            wf_ready: false,
            wf_col: [0; 12],
        }
    }

    /// Initialize a `Receiver` in-place without constructing a ~13 KB temporary on the stack.
    ///
    /// # Safety
    /// `dst` must be valid for writes and properly aligned for `Self`.
    pub unsafe fn init_in_place(dst: *mut Self) {
        core::ptr::write_bytes(dst, 0, 1);
        (*dst).tone_q_off = compute_tone_q_off(DEFAULT_SPACING_HZ);
    }

    /// Reset state at the UTC `10.0 s` slot boundary (`t = 0.00 s`).
    pub fn reset(&mut self) {
        self.decim.reset();
        self.pcm_ring = [0; SYM_SAMPLES];
        self.pcm_pos = 0;
        self.decim_count = 0;
        self.work.fill(0);
        self.tracks = [Track::empty(); MAX_TRACKS];
        self.acquired = false;
        self.wf_ready = false;
        self.wf_col = [0; 12];
    }

    /// Take the latest 24-bin (`586..2,344 Hz`) packed 4-bit waterfall column (emitted every `200 ms`).
    pub fn take_wf_col(&mut self) -> Option<[u8; 12]> {
        if self.wf_ready {
            self.wf_ready = false;
            Some(self.wf_col)
        } else {
            None
        }
    }

    /// Diagnostic hook: bypass blind `S0 + S1` acquisition and pin track 0 to the nearest `(start_sym, phase, q0)`
    /// with sub-grid `(dt_q4, df_q2)` for a known burst start time (`start_ms`) and base tone (`audio_hz`).
    pub fn set_oracle_track(&mut self, start_ms: u32, audio_hz: u32) {
        let h = ((start_ms + 20) / 40) as usize;
        let start_sym = (h / 2) as u8;
        let phase = (h % 2) as u8;
        let dt_ms = start_ms as i32 - (h as i32 * 40);
        let dt_q4 = ((dt_ms * 6) / 4).clamp(-30, 30) as i8; // 6 samples/ms / 4
        let half_bin = ((audio_hz as u64 * (FFT_N as u64 * 2) + (PROC_RATE_HZ as u64 / 2)) / PROC_RATE_HZ as u64) as usize;
        let q0 = half_bin.saturating_sub(2 * FIRST_BIN).min((N_BINS - 2) * 2) as u16;
        let q0_hz = (2 * FIRST_BIN as u32 + q0 as u32) as f32 * (PROC_RATE_HZ as f32 / (2.0 * FFT_N as f32));
        let df_q2 = ((audio_hz as f32 - q0_hz) * 4.0).clamp(-24.0, 24.0) as i8;
        self.tracks[0] = Track {
            active: true,
            start_sym,
            phase,
            dt_q4,
            q0,
            df_q2,
            ref_first_d: REF_START_DATA as u8,
            ..Track::empty()
        };
        self.work.fill(0);
        self.acquired = true;
    }

    #[inline]
    fn ring_offset(sym_mod: usize, phase: usize, bin: usize) -> usize {
        (sym_mod * 2 + phase) * N_BINS + bin
    }

    #[inline]
    fn write_ring_bins(&mut self, sym_mod: usize, phase: usize, bins: &[u8; N_BINS]) {
        let base = Self::ring_offset(sym_mod, phase, 0) >> 1;
        for i in 0..(N_BINS / 2) {
            self.work[base + i] = i16::from_le_bytes([bins[2 * i], bins[2 * i + 1]]);
        }
    }

    #[inline]
    fn read_ring_bin(&self, sym_mod: usize, phase: usize, bin: usize) -> u8 {
        let off = Self::ring_offset(sym_mod, phase, bin);
        let [lo, hi] = self.work[off >> 1].to_le_bytes();
        if (off & 1) == 0 { lo } else { hi }
    }

    #[inline]
    fn sample_ring_tone(&self, sym_mod: usize, phase: usize, q0: u16, tone: usize) -> i32 {
        let q = (q0 + self.tone_q_off[tone]) as usize;
        let b = q >> 1;
        if b >= N_BINS {
            return 0;
        }
        let v0 = self.read_ring_bin(sym_mod, phase, b) as i32;
        if (q & 1) == 0 || b + 1 >= N_BINS {
            v0
        } else {
            (v0 + self.read_ring_bin(sym_mod, phase, b + 1) as i32 + 1) >> 1
        }
    }

    #[inline]
    fn read_ring_bins(&self, sym_mod: usize, phase: usize) -> [u8; N_BINS] {
        let base = Self::ring_offset(sym_mod, phase, 0) >> 1;
        let mut out = [0u8; N_BINS];
        for i in 0..(N_BINS / 2) {
            let [lo, hi] = self.work[base + i].to_le_bytes();
            out[2 * i] = lo;
            out[2 * i + 1] = hi;
        }
        out
    }

    /// Feed signed 16-bit PCM samples at `24,000 Hz` (e.g. `192`-sample `8 ms` DSP blocks).
    pub fn push_samples(&mut self, samples: &[i16]) {
        for &x in samples {
            if let Some(y) = self.decim.push(x, &DECIM_TAPS, DECIM_FACTOR) {
                self.pcm_ring[self.pcm_pos as usize] = y;
                self.pcm_pos = if self.pcm_pos as usize + 1 == SYM_SAMPLES { 0 } else { self.pcm_pos + 1 };
                if self.acquired {
                    self.work[(self.decim_count as usize) % HIST_LEN] = y;
                }
                self.decim_count += 1;

                if self.decim_count >= SYM_SAMPLES as u32
                    && (self.decim_count - SYM_SAMPLES as u32).is_multiple_of(HALF_SAMPLES as u32)
                {
                    let h = ((self.decim_count - SYM_SAMPLES as u32) / HALF_SAMPLES as u32) as usize;
                    let sym = h / 2;
                    let phase = h % 2;
                    self.on_half_symbol(sym, phase);
                }
            }
        }
    }

    #[inline(never)]
    fn on_half_symbol(&mut self, sym: usize, phase: usize) {
        let mut z = [0i16; FFT_N];
        let mut idx = self.pcm_pos as usize;
        for (n, &w) in HANN_480.iter().enumerate() {
            let s = self.pcm_ring[idx] as i32;
            idx = if idx + 1 == SYM_SAMPLES { 0 } else { idx + 1 };
            z[n] = ((s * w as i32 + 16_384) >> 15) as i16;
        }
        let shift = cfft(&mut z, FFT_M);

        let mut cur_bins = [0u8; N_BINS];
        let mut out_i = 0usize;
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            cur_bins[out_i] = lp.clamp(0, 255) as u8;
            out_i += 1;
        });

        // Emit a 24-bin (`586..2,344 Hz`) packed 4-bit waterfall column every 5 half-symbols (`200 ms`).
        if (sym * 2 + phase).is_multiple_of(5) {
            let mut sum = 0u32;
            for &b in &cur_bins {
                sum += b as u32;
            }
            let floor = (sum / N_BINS as u32) as i32;
            let mut col = [0u8; 12];
            for r in 0..24usize {
                let lo = r * N_BINS / 24;
                let hi = ((r + 1) * N_BINS / 24).max(lo + 1);
                let mut pk = 0u8;
                for &b in &cur_bins[lo..hi] {
                    if b > pk {
                        pk = b;
                    }
                }
                // Map `floor..floor+45` (~0..17 dB above local passband floor) into `0..=15`.
                let lvl = (((pk as i32 - floor + 2).max(0) * 15) / 42).clamp(0, 15) as u8;
                if (r & 1) == 0 {
                    col[r >> 1] = lvl;
                } else {
                    col[r >> 1] |= lvl << 4;
                }
            }
            self.wf_col = col;
            self.wf_ready = true;
        }

        if !self.acquired {
            self.write_ring_bins(sym % RING_SYMS, phase, &cur_bins);
            // Spread the 18 `(s, phase)` acquisition searches across `sym = 24..=32` (1 per 40 ms half-symbol)
            // so no single 8 ms audio DMA half ever stalls. `s = sym - 23` has all of `S0 (s..s+6)` and
            // `S1 (s+18..s+24)` in the ring buffer right as `sym` finishes arriving!
            if (24..=ACQUIRE_AT_SYM).contains(&sym) {
                let s = sym - 23; // 1..=9 == MIN_START_SYM..=MAX_START_SYM
                if sym == 24 && phase == 0 {
                    self.acq_scratch_init();
                }
                self.acquire_step(s, phase);
            }
            if sym == ACQUIRE_AT_SYM && phase == 1 {
                self.acquire_finish();
                self.acquired = true;
            }
            return;
        }

        for tr_i in 0..MAX_TRACKS {
            if !self.tracks[tr_i].active || self.tracks[tr_i].phase as usize != phase {
                continue;
            }
            let start = self.tracks[tr_i].start_sym as usize;
            if sym < start {
                continue;
            }
            let rel = sym - start;
            if rel < N_SYMBOLS {
                self.feed_track_symbol(tr_i, rel, &cur_bins);
            }
        }

        // Streaming rectangular Goertzel demodulation on the 140 ms circular PCM ring (`work[0..HIST_LEN]`).
        // Evaluated 1 half-symbol (+40 ms = 240 samples) after each track's nominal symbol end so any
        // `dt_samp in [-120..+120]` (`+-20 ms`) window lies completely inside `work[0..HIST_LEN]`.
        let h_cur = sym * 2 + phase;
        if h_cur > 0 && self.decim_count as usize >= HIST_LEN {
            let h_prev = h_cur - 1;
            let df_tone = if self.tone_q_off[3] == 26 {
                DEFAULT_SPACING_HZ as f32
            } else {
                (self.tone_q_off[3] as f32) * (PROC_RATE_HZ as f32 / (6.0 * FFT_N as f32))
            };
            for tr_i in 0..MAX_TRACKS {
                if !self.tracks[tr_i].active {
                    continue;
                }
                let h_start = self.tracks[tr_i].start_sym as usize * 2 + self.tracks[tr_i].phase as usize;
                if h_prev < h_start || ((h_prev - h_start) & 1) != 0 {
                    continue;
                }
                let rel = (h_prev - h_start) >> 1;
                if rel >= N_SYMBOLS {
                    continue;
                }
                let is_s2 = rel >= 104;
                let d_opt = if !is_s2 && rel >= 24 { Some(12 + (rel - 24)) } else { None };
                if !is_s2 && d_opt.is_none_or(|d| d >= N_DATA_SYMS) {
                    continue;
                }
                let dt_samp = self.tracks[tr_i].dt_q4 as i32 * 4;
                let win_start = (self.decim_count as i32 - 720 + dt_samp) as usize % HIST_LEN;
                let f0 = self.tracks[tr_i].base_freq_hz();
                let mut e = [0.0f32; 4];
                for (t, slot) in e.iter_mut().enumerate() {
                    *slot = goertzel_energy(&self.work[..HIST_LEN], win_start, f0 + t as f32 * df_tone);
                }
                if is_s2 {
                    let w = S2[rel - 104] as usize;
                    let base = S2_BASE + tr_i * 4;
                    let mut on = read_f32_work(&self.work, base);
                    let mut off = read_f32_work(&self.work, base + 2);
                    for (t, &et) in e.iter().enumerate() {
                        if t == w {
                            on += et;
                        } else {
                            off += et;
                        }
                    }
                    write_f32_work(&mut self.work, base, on);
                    write_f32_work(&mut self.work, base + 2, off);
                } else if let Some(d) = d_opt {
                    let base = REF_BASE + tr_i * REF_STRIDE + (d - REF_START_DATA) * 4;
                    for (t, &et) in e.iter().enumerate() {
                        self.work[base + t] = pack_energy_q8(et) as i16;
                    }
                    if (d as u8) < self.tracks[tr_i].ref_first_d {
                        self.tracks[tr_i].ref_first_d = d as u8;
                    }
                }
            }
        }
    }

    #[inline]
    fn sample_tone(bins: &[u8; N_BINS], q0: u16, tone_q_off: &[u16; 4], tone: usize) -> i32 {
        let q = (q0 + tone_q_off[tone]) as usize;
        let b = q >> 1;
        if b >= N_BINS {
            return 0;
        }
        if (q & 1) == 0 || b + 1 >= N_BINS {
            bins[b] as i32
        } else {
            (bins[b] as i32 + bins[b + 1] as i32 + 1) >> 1
        }
    }

    // During acquisition (`sym = 24..=32`), `self.tracks[0..6].llr_zero` (unused until `acquire_finish`)
    // stores `best_score[0..300]` (600 B across tracks 0..4) and `best_sp[0..300]` (300 B across tracks 4..6).
    fn acq_scratch_init(&mut self) {
        let min_bytes = i16::MIN.to_le_bytes();
        for q in 0..300usize {
            let b = q * 2;
            self.tracks[b / N_BITS].llr_zero[b % N_BITS] = min_bytes[0] as i8;
            self.tracks[(b + 1) / N_BITS].llr_zero[(b + 1) % N_BITS] = min_bytes[1] as i8;
            self.tracks[4 + q / N_BITS].llr_zero[q % N_BITS] = 0;
        }
    }

    #[inline]
    fn acq_get_score(&self, q: usize) -> i16 {
        let b = q * 2;
        let lo = self.tracks[b / N_BITS].llr_zero[b % N_BITS] as u8;
        let hi = self.tracks[(b + 1) / N_BITS].llr_zero[(b + 1) % N_BITS] as u8;
        i16::from_le_bytes([lo, hi])
    }

    #[inline]
    fn acq_set(&mut self, q: usize, sc: i16, sp: u8) {
        let b = q * 2;
        let bytes = sc.to_le_bytes();
        self.tracks[b / N_BITS].llr_zero[b % N_BITS] = bytes[0] as i8;
        self.tracks[(b + 1) / N_BITS].llr_zero[(b + 1) % N_BITS] = bytes[1] as i8;
        self.tracks[4 + q / N_BITS].llr_zero[q % N_BITS] = sp as i8;
    }

    #[inline]
    fn expand_half_bins(bins: &[u8; N_BINS]) -> [u8; N_BINS * 2] {
        let mut h = [0u8; N_BINS * 2];
        for b in 0..N_BINS {
            let v0 = bins[b] as u16;
            h[b * 2] = v0 as u8;
            h[b * 2 + 1] = if b + 1 < N_BINS {
                ((v0 + bins[b + 1] as u16 + 1) >> 1) as u8
            } else {
                v0 as u8
            };
        }
        h
    }

    /// Score one `(s, phase)` pair against all `q0 in 0..n_q` using one 324 B row at a time (~1.5 ms on 120 MHz M4).
    #[inline(never)]
    fn acquire_step(&mut self, s: usize, phase: usize) {
        let max_q0 = ((N_BINS - 2) * 2).saturating_sub(self.tone_q_off[3] as usize) as u16;
        let n_q = (max_q0 as usize + 1).min(300);
        let o0 = self.tone_q_off[0] as usize;
        let o1 = self.tone_q_off[1] as usize;
        let o2 = self.tone_q_off[2] as usize;
        let o3 = self.tone_q_off[3] as usize;
        let mut contrast = [0i16; 300];
        let mut raw_sum = [0u16; 300];
        for (k, &w) in S0.iter().enumerate() {
            let b0 = self.read_ring_bins((s + k) % RING_SYMS, phase);
            let row = Self::expand_half_bins(&b0);
            for q0 in 0..n_q {
                let e0 = row[q0 + o0] as i32;
                let e1 = row[q0 + o1] as i32;
                let e2 = row[q0 + o2] as i32;
                let e3 = row[q0 + o3] as i32;
                let arr = [e0, e1, e2, e3];
                let want = arr[(w as usize) & 3];
                contrast[q0] += (4 * want - (e0 + e1 + e2 + e3)).clamp(-48, 48) as i16;
                raw_sum[q0] += want as u16;
            }
        }
        for (k, &w) in S1.iter().enumerate() {
            let b1 = self.read_ring_bins((s + 18 + k) % RING_SYMS, phase);
            let row = Self::expand_half_bins(&b1);
            for q0 in 0..n_q {
                let e0 = row[q0 + o0] as i32;
                let e1 = row[q0 + o1] as i32;
                let e2 = row[q0 + o2] as i32;
                let e3 = row[q0 + o3] as i32;
                let arr = [e0, e1, e2, e3];
                let want = arr[(w as usize) & 3];
                contrast[q0] += (4 * want - (e0 + e1 + e2 + e3)).clamp(-48, 48) as i16;
                raw_sum[q0] += want as u16;
            }
        }
        let sp = ((s as u8) << 1) | (phase as u8);
        for q0 in 0..n_q {
            let score = (contrast[q0] as i32) * 8 + ((raw_sum[q0] as i32) >> 4);
            let sc = score.clamp(i16::MIN as i32 + 1, i16::MAX as i32) as i16;
            if sc > self.acq_get_score(q0) {
                self.acq_set(q0, sc, sp);
            }
        }
    }

    #[inline(never)]
    fn acquire_finish(&mut self) {
        let max_q0 = ((N_BINS - 2) * 2).saturating_sub(self.tone_q_off[3] as usize) as u16;
        let n_q = (max_q0 as usize + 1).min(300);
        let mut best_score = [i16::MIN; 300];
        let mut best_sp = [0u8; 300];
        for q in 0..n_q {
            best_score[q] = self.acq_get_score(q);
            best_sp[q] = self.tracks[4 + q / N_BITS].llr_zero[q % N_BITS] as u8;
        }
        for t in &mut self.tracks {
            *t = Track::empty();
        }

        let mut chosen_n = 0usize;
        for slot in 0..MAX_TRACKS {
            let min_sep: usize = if slot < MAX_TRACKS - 2 { 8 } else { 32 };
            let mut pick_q = None;
            let mut pick_sc = 288i16;
            for q0 in 0..n_q {
                let sc = best_score[q0];
                if sc <= pick_sc {
                    continue;
                }
                if q0 > 0 && best_score[q0 - 1] > sc {
                    continue;
                }
                if q0 + 1 < n_q && best_score[q0 + 1] >= sc {
                    continue;
                }
                let mut too_close = false;
                for prev in &self.tracks[..chosen_n] {
                    if (prev.q0 as usize).abs_diff(q0) < min_sep {
                        too_close = true;
                        break;
                    }
                }
                if !too_close {
                    pick_sc = sc;
                    pick_q = Some(q0);
                }
            }
            let Some(q0) = pick_q else {
                continue;
            };
            let sp = best_sp[q0];
            let start_sym = sp >> 1;
            let phase = sp & 1;
            self.tracks[chosen_n] = Track {
                active: true,
                start_sym,
                phase,
                q0: q0 as u16,
                ..Track::empty()
            };
            let s_usize = start_sym as usize;
            let ph_usize = phase as usize;
            // Refine sub-bin frequency (`df_q2`) and sub-half-symbol timing (`dt_q4`) via 3-point
            // parabolic interpolation on `S0 + S1` log-power in `work` (direct single-bin reads).
            let sync_log_sum = |s_a: usize, ph_a: usize, q_a: u16| -> f32 {
                let mut acc = 0i32;
                for (k, &w) in S0.iter().enumerate() {
                    acc += self.sample_ring_tone((s_a + k) % RING_SYMS, ph_a, q_a, w as usize);
                }
                for (k, &w) in S1.iter().enumerate() {
                    acc += self.sample_ring_tone((s_a + 18 + k) % RING_SYMS, ph_a, q_a, w as usize);
                }
                acc as f32
            };
            let q_u16 = q0 as u16;
            let df_q2 = if q_u16 >= 2 && (q_u16 as usize + 2) < n_q {
                let ym = sync_log_sum(s_usize, ph_usize, q_u16 - 2);
                let y0 = sync_log_sum(s_usize, ph_usize, q_u16);
                let yp = sync_log_sum(s_usize, ph_usize, q_u16 + 2);
                let denom = ym - 2.0 * y0 + yp;
                if denom < -1e-3 {
                    let delta_bins = (0.5 * (ym - yp) / denom).clamp(-0.5, 0.5);
                    let df_hz = delta_bins * (PROC_RATE_HZ as f32 / FFT_N as f32);
                    let q2 = (df_hz * 4.0) + if df_hz >= 0.0 { 0.5 } else { -0.5 };
                    (q2 as i32).clamp(-24, 24) as i8
                } else {
                    0
                }
            } else {
                0
            };
            let h_start = s_usize * 2 + ph_usize;
            let dt_q4 = if (2..MAX_START_SYM * 2 + 1).contains(&h_start) {
                let hm = h_start - 1;
                let hp = h_start + 1;
                let tm = sync_log_sum(hm >> 1, hm & 1, q_u16);
                let t0 = sync_log_sum(s_usize, ph_usize, q_u16);
                let tp = sync_log_sum(hp >> 1, hp & 1, q_u16);
                let denom = tm - 2.0 * t0 + tp;
                if denom < -1e-3 {
                    let delta_half = (0.5 * (tm - tp) / denom).clamp(-0.5, 0.5);
                    let dt_samp = delta_half * (HALF_SAMPLES as f32);
                    let q4 = (dt_samp * 0.25) + if dt_samp >= 0.0 { 0.5 } else { -0.5 };
                    (q4 as i32).clamp(-30, 30) as i8
                } else {
                    0
                }
            } else {
                0
            };
            self.tracks[chosen_n].df_q2 = df_q2;
            self.tracks[chosen_n].dt_q4 = dt_q4;

            for sym in s_usize..=ACQUIRE_AT_SYM {
                let rel = sym - s_usize;
                if rel < N_SYMBOLS {
                    let bins = self.read_ring_bins(sym % RING_SYMS, ph_usize);
                    self.feed_track_symbol(chosen_n, rel, &bins);
                }
            }
            // Upgrade early backfilled data symbols (`rel <= ACQUIRE_AT_SYM - start_sym`) to `ln I_0`
            // linear-power LLRs scaled to match the streaming rectangular Goertzel LLRs (`* 12.0`).
            let off_sum = self.tracks[chosen_n].off_pwr_sum as f32;
            let on_sum = self.tracks[chosen_n].on_pwr_sum as f32;
            if off_sum > 0.0 {
                let n0 = (off_sum / 36.0).max(1.0);
                let e_on = on_sum / 12.0;
                let gamma = ((e_on / n0) - 1.0).clamp(0.3, 20.0);
                for sym in s_usize..=ACQUIRE_AT_SYM {
                    let rel = sym - s_usize;
                    if rel < 6 || (18..24).contains(&rel) || rel >= 104 {
                        continue;
                    }
                    let d = if rel < 18 { rel - 6 } else { 12 + (rel - 24) };
                    if d >= N_DATA_SYMS {
                        continue;
                    }
                    let sym_mod = sym % RING_SYMS;
                    let mut lp = [0.0f32; 4];
                    for (t, slot) in lp.iter_mut().enumerate() {
                        let b = self.sample_ring_tone(sym_mod, ph_usize, q_u16, t).clamp(0, 255) as u8;
                        let ratio = lin_power(b) as f32 / n0;
                        *slot = ln_bessel_i0_f32(2.0 * sqrt_f32(gamma * ratio));
                    }
                    let llr0 = (log_sum_exp_f32(lp[0], lp[1]) - log_sum_exp_f32(lp[2], lp[3])) * 12.0;
                    let llr1 = (log_sum_exp_f32(lp[0], lp[3]) - log_sum_exp_f32(lp[1], lp[2])) * 12.0;
                    let r0 = llr0 + if llr0 >= 0.0 { 0.5 } else { -0.5 };
                    let r1 = llr1 + if llr1 >= 0.0 { 0.5 } else { -0.5 };
                    self.tracks[chosen_n].llr_zero[2 * d] = (r0 as i32).clamp(-120, 120) as i8;
                    self.tracks[chosen_n].llr_zero[2 * d + 1] = (r1 as i32).clamp(-120, 120) as i8;
                }
            }
            chosen_n += 1;
        }

        // Transition `self.work` from the acquisition spectrogram to the streaming PCM ring + Goertzel store.
        self.work.fill(0);
        if self.decim_count as usize >= SYM_SAMPLES {
            let start_cnt = self.decim_count as usize - SYM_SAMPLES;
            let mut r_idx = self.pcm_pos as usize;
            for i in 0..SYM_SAMPLES {
                self.work[(start_cnt + i) % HIST_LEN] = self.pcm_ring[r_idx];
                r_idx += 1;
                if r_idx == SYM_SAMPLES {
                    r_idx = 0;
                }
            }
        }
    }

    #[inline]
    fn contrast_sym(bins: &[u8; N_BINS], q0: u16, tone_q_off: &[u16; 4], expected: usize) -> (i32, i32) {
        let e0 = Self::sample_tone(bins, q0, tone_q_off, 0);
        let e1 = Self::sample_tone(bins, q0, tone_q_off, 1);
        let e2 = Self::sample_tone(bins, q0, tone_q_off, 2);
        let e3 = Self::sample_tone(bins, q0, tone_q_off, 3);
        let arr = [e0, e1, e2, e3];
        let sum = e0 + e1 + e2 + e3;
        let want = arr[expected & 3];
        ((4 * want - sum).clamp(-48, 48), want)
    }

    #[inline]
    fn extract_dibit_llr(bins: &[u8; N_BINS], q: u16, tone_q_off: &[u16; 4]) -> (i8, i8) {
        let e0 = Self::sample_tone(bins, q, tone_q_off, 0);
        let e1 = Self::sample_tone(bins, q, tone_q_off, 1);
        let e2 = Self::sample_tone(bins, q, tone_q_off, 2);
        let e3 = Self::sample_tone(bins, q, tone_q_off, 3);
        let b0 = (e0.max(e1) - e2.max(e3)).clamp(-24, 24) as i8;
        let b1 = (e0.max(e3) - e1.max(e2)).clamp(-24, 24) as i8;
        (b0, b1)
    }

    fn feed_track_symbol(&mut self, tr_i: usize, rel_sym: usize, bins: &[u8; N_BINS]) {
        let q0 = self.tracks[tr_i].q0;

        // Tail sync S2 (`104..110`): score the 3 drift endpoints `[-2, 0, +2]` half-bins (`-11.7, 0, +11.7 Hz`),
        // including the raw energy tie-breaker so high-SNR frames don't tie at the +48 contrast ceiling.
        if rel_sym >= 104 {
            let k = rel_sym - 104;
            let w = S2[k] as usize;
            let q_neg = q0.saturating_sub(2);
            let q_pos = q0.saturating_add(2);
            let (cn, wn) = Self::contrast_sym(bins, q_neg, &self.tone_q_off, w);
            let (cz, wz) = Self::contrast_sym(bins, q0, &self.tone_q_off, w);
            let (cp, wp) = Self::contrast_sym(bins, q_pos, &self.tone_q_off, w);
            self.tracks[tr_i].s2_scores[0] = self.tracks[tr_i].s2_scores[0].saturating_add((cn * 4 + (wn >> 2)) as i16);
            self.tracks[tr_i].s2_scores[1] = self.tracks[tr_i].s2_scores[1].saturating_add((cz * 4 + (wz >> 2)) as i16);
            self.tracks[tr_i].s2_scores[2] = self.tracks[tr_i].s2_scores[2].saturating_add((cp * 4 + (wp >> 2)) as i16);
            return;
        }

        // Early sync symbols `S0` (`0..6`) and `S1` (`18..24`): accumulate unbiased linear power on active vs
        // inactive sync tones for calibrated 2,500 Hz SNR estimation.
        if rel_sym < 6 || (18..24).contains(&rel_sym) {
            let w = if rel_sym < 6 { S0[rel_sym] } else { S1[rel_sym - 18] } as usize;
            for t in 0..4 {
                let lp = Self::sample_tone(bins, q0, &self.tone_q_off, t).clamp(0, 255) as u8;
                let p = lin_power(lp);
                if t == w {
                    self.tracks[tr_i].on_pwr_sum = self.tracks[tr_i].on_pwr_sum.saturating_add(p);
                } else {
                    self.tracks[tr_i].off_pwr_sum = self.tracks[tr_i].off_pwr_sum.saturating_add(p);
                }
            }
            return;
        }

        let d = if rel_sym < 18 { rel_sym - 6 } else { 12 + (rel_sym - 24) };
        if d >= N_DATA_SYMS {
            return;
        }

        let (b0, b1) = Self::extract_dibit_llr(bins, q0, &self.tone_q_off);
        self.tracks[tr_i].llr_zero[2 * d] = b0;
        self.tracks[tr_i].llr_zero[2 * d + 1] = b1;

        // Drift-divergent data symbols (`d = 36..92`, `rel_sym = 48..104`): record negative and positive linear
        // drift trajectories using `drift_step_half_bins(rel_sym)` (`1` half-bin on `48..83`, `2` on `83..104`).
        if d >= DRIFT_START_DATA {
            let step = drift_step_half_bins(rel_sym);
            let (nb0, nb1) = Self::extract_dibit_llr(bins, q0.saturating_sub(step), &self.tone_q_off);
            let (pb0, pb1) = Self::extract_dibit_llr(bins, q0.saturating_add(step), &self.tone_q_off);
            let ti = d - DRIFT_START_DATA;
            self.tracks[tr_i].llr_neg_tail[ti] = pack_dibit_i4(nb0, nb1);
            self.tracks[tr_i].llr_pos_tail[ti] = pack_dibit_i4(pb0, pb1);
        }
    }

    /// Diagnostic hook: return `(start_ms, audio_hz)` of admitted track `tr_i` on the receiver's `40 ms` / half-bin grid.
    pub fn track_coords(&self, tr_i: usize) -> Option<(u32, u32)> {
        if tr_i >= MAX_TRACKS || !self.tracks[tr_i].active {
            return None;
        }
        let tr = &self.tracks[tr_i];
        let start_ms = tr.start_sym as u32 * 80 + tr.phase as u32 * 40;
        let audio_hz = ((2 * FIRST_BIN as u32 + tr.q0 as u32) * (PROC_RATE_HZ / 2) + 256) / FFT_N as u32;
        Some((start_ms, audio_hz))
    }

    /// Diagnostic hook: export track `tr_i`'s zero-drift bit LLRs in canonical LDPC codeword order (`184` bits).
    pub fn track_codeword_llrs(&self, tr_i: usize) -> Option<[i8; N_BITS]> {
        if tr_i >= MAX_TRACKS || !self.tracks[tr_i].active {
            return None;
        }
        let mut out = [0i8; N_BITS];
        for code_i in 0..N_BITS {
            out[code_i] = self.tracks[tr_i].llr_zero[interleave_idx(code_i)];
        }
        Some(out)
    }

    /// Diagnostic snapshot: `(completed_symbols, active_tracks, acquired)`.
    pub fn diag(&self) -> (u16, u8, bool) {
        let syms = (self.decim_count / SYM_SAMPLES as u32).min(u16::MAX as u32) as u16;
        let active = self.tracks.iter().filter(|t| t.active).count() as u8;
        (syms, active, self.acquired)
    }

    /// Run serial normalized min-sum LDPC + CRC-14 decoding across all admitted tracks at end of slot, reusing
    /// `self.work[BP_BASE..]` for `post` (`184`) and `c2v` (`525`) scratch so `REF_BASE..S2_BASE` stays intact.
    pub fn finish_slot(&mut self, out: &mut [Decoded; MAX_DECODED]) -> usize {
        self.finish_slot_with_soft(out, None)
    }

    /// Same as [`Self::finish_slot`], optionally exporting the highest-scoring failed-CRC candidate as a
    /// `92`-byte 4-bit [`SoftCandidate`] for Cooperative RX capture / combining.
    pub fn finish_slot_with_soft(
        &mut self,
        out: &mut [Decoded; MAX_DECODED],
        mut soft_out: Option<&mut Option<SoftCandidate>>,
    ) -> usize {
        if let Some(s) = soft_out.as_deref_mut() {
            *s = None;
        }
        let mut best_soft_score = i16::MIN;
        let mut raw_llr = [0i8; N_BITS];
        let mut deinter = [0i8; N_BITS];
        let mut n_out = 0usize;

        for tr_i in 0..MAX_TRACKS {
            if !self.tracks[tr_i].active || n_out >= MAX_DECODED {
                continue;
            }
            let mut decoded_frame = None;

            // Priority 1: refined rectangular-windowed Goertzel `ln I_0` LLRs (using `S2` local noise `N0` & `gamma`).
            let s2_base = S2_BASE + tr_i * 4;
            let s2_on = read_f32_work(&self.work, s2_base);
            let s2_off = read_f32_work(&self.work, s2_base + 2);
            let ref_first = self.tracks[tr_i].ref_first_d as usize;
            if s2_off > 0.0 && ref_first < N_DATA_SYMS {
                let n0 = (s2_off / 18.0).max(1.0);
                let e_on = s2_on / 6.0;
                let gamma = ((e_on / n0) - 1.0).clamp(0.3, 20.0);
                let n0_q8 = pack_energy_q8(n0) as f32;
                raw_llr.copy_from_slice(&self.tracks[tr_i].llr_zero);
                for d in ref_first..N_DATA_SYMS {
                    let base = REF_BASE + tr_i * REF_STRIDE + (d - REF_START_DATA) * 4;
                    let mut lp = [0.0f32; 4];
                    for (t, slot) in lp.iter_mut().enumerate() {
                        let e_q8 = self.work[base + t] as u16 as f32;
                        let ratio = exp2_f32((e_q8 - n0_q8) * (1.0 / 256.0));
                        *slot = ln_bessel_i0_f32(2.0 * sqrt_f32(gamma * ratio));
                    }
                    let llr0 = (log_sum_exp_f32(lp[0], lp[1]) - log_sum_exp_f32(lp[2], lp[3])) * 12.0;
                    let llr1 = (log_sum_exp_f32(lp[0], lp[3]) - log_sum_exp_f32(lp[1], lp[2])) * 12.0;
                    let r0 = llr0 + if llr0 >= 0.0 { 0.5 } else { -0.5 };
                    let r1 = llr1 + if llr1 >= 0.0 { 0.5 } else { -0.5 };
                    raw_llr[2 * d] = (r0 as i32).clamp(-120, 120) as i8;
                    raw_llr[2 * d + 1] = (r1 as i32).clamp(-120, 120) as i8;
                }
                for code_i in 0..N_BITS {
                    deinter[code_i] = raw_llr[interleave_idx(code_i)];
                }
                let (post_slice, rest) = self.work[BP_BASE..].split_at_mut(N_BITS);
                let post: &mut [i16; N_BITS] = post_slice.try_into().unwrap();
                let c2v: &mut [i16; N_EDGES] = (&mut rest[..N_EDGES]).try_into().unwrap();
                if let Some(bits78) = ldpc::decode_bp_with_slices(&deinter, post, c2v, LDPC_MAX_ITER) {
                    if let Some(f) = Frame::from_bits78(&bits78) {
                        decoded_frame = Some(f);
                    }
                }
            }

            // Priority 2: fallback to 3-branch linear drift decoder (`-11.7, 0, +11.7 Hz`) ranked by `S2`.
            if decoded_frame.is_none() {
                let sc = [
                    self.tracks[tr_i].s2_scores[0],
                    self.tracks[tr_i].s2_scores[1].saturating_add(16),
                    self.tracks[tr_i].s2_scores[2],
                ];
                let mut order = [1usize, 0, 2];
                if sc[order[1]] > sc[order[0]] {
                    order.swap(0, 1);
                }
                if sc[order[2]] > sc[order[1]] {
                    order.swap(1, 2);
                    if sc[order[1]] > sc[order[0]] {
                        order.swap(0, 1);
                    }
                }

                let (post_slice, rest) = self.work[BP_BASE..].split_at_mut(N_BITS);
                let post: &mut [i16; N_BITS] = post_slice.try_into().unwrap();
                let c2v: &mut [i16; N_EDGES] = (&mut rest[..N_EDGES]).try_into().unwrap();
                for &branch in &order {
                    raw_llr.copy_from_slice(&self.tracks[tr_i].llr_zero);
                    if branch == 0 {
                        for (ti, &pk) in self.tracks[tr_i].llr_neg_tail.iter().enumerate() {
                            let (b0, b1) = unpack_dibit_i4(pk);
                            let bit_i = (DRIFT_START_DATA + ti) * 2;
                            raw_llr[bit_i] = b0;
                            raw_llr[bit_i + 1] = b1;
                        }
                    } else if branch == 2 {
                        for (ti, &pk) in self.tracks[tr_i].llr_pos_tail.iter().enumerate() {
                            let (b0, b1) = unpack_dibit_i4(pk);
                            let bit_i = (DRIFT_START_DATA + ti) * 2;
                            raw_llr[bit_i] = b0;
                            raw_llr[bit_i + 1] = b1;
                        }
                    }
                    for code_i in 0..N_BITS {
                        deinter[code_i] = raw_llr[interleave_idx(code_i)];
                    }
                    if let Some(bits78) = ldpc::decode_bp_with_slices(&deinter, post, c2v, LDPC_MAX_ITER) {
                        if let Some(f) = Frame::from_bits78(&bits78) {
                            decoded_frame = Some(f);
                            break;
                        }
                    }
                }
            }

            let Some(frame) = decoded_frame else {
                if let Some(slot_soft) = soft_out.as_deref_mut() {
                    let tr = &self.tracks[tr_i];
                    let sync_sc = tr.s2_scores[1].max(tr.s2_scores[0]).max(tr.s2_scores[2]);
                    if sync_sc > best_soft_score {
                        best_soft_score = sync_sc;
                        let audio_hz = (tr.base_freq_hz() + 0.5).clamp(0.0, 6000.0) as u16;
                        let time_offset_ms = ((tr.start_sym as i16 - NOMINAL_START_SYM as i16) * 80)
                            + (tr.phase as i16 * 40)
                            + ((tr.dt_q4 as i16 * 2) / 3);
                        let snr_db = estimate_snr_2500_db(tr.on_pwr_sum, tr.off_pwr_sum);
                        *slot_soft = Some(SoftCandidate {
                            audio_hz,
                            time_offset_ms,
                            snr_db,
                            sync_score: sync_sc,
                            llrs_4bit: SoftCandidate::pack_llrs(&deinter),
                        });
                    }
                }
                continue;
            };
            if out[..n_out].iter().any(|d| d.frame == frame) {
                continue;
            }

            let tr = &self.tracks[tr_i];
            let audio_hz = (((2 * FIRST_BIN as u32 + tr.q0 as u32) * (PROC_RATE_HZ / 2) + 256) / FFT_N as u32) as u16;
            let time_offset_ms = ((tr.start_sym as i16 - NOMINAL_START_SYM as i16) * 80) + (tr.phase as i16 * 40);
            let snr_db = estimate_snr_2500_db(tr.on_pwr_sum, tr.off_pwr_sum);

            out[n_out] = Decoded { frame, audio_hz, time_offset_ms, snr_db };
            n_out += 1;
        }
        n_out
    }

    #[cfg(test)]
    pub(crate) fn top_track_drift_info(&self) -> Option<([i16; 3], usize, usize)> {
        let tr = self.tracks.iter().find(|t| t.active)?;
        let mut diff_step1 = 0usize;
        let mut diff_step2 = 0usize;
        for ti in 0..TAIL_DATA_SYMS {
            let d = DRIFT_START_DATA + ti;
            let (pb0, pb1) = unpack_dibit_i4(tr.llr_pos_tail[ti]);
            let (z0, z1) = unpack_dibit_i4(pack_dibit_i4(tr.llr_zero[2 * d], tr.llr_zero[2 * d + 1]));
            if (pb0, pb1) != (z0, z1) {
                if d < 71 {
                    diff_step1 += 1;
                } else {
                    diff_step2 += 1;
                }
            }
        }
        Some((tr.s2_scores, diff_step1, diff_step2))
    }
}

/// Convert accumulated linear sync power (`on_pwr_sum` across 12 active sync bins, `off_pwr_sum` across 36 inactive
/// sync bins) into unbiased SNR in dB referenced to a `2,500 Hz` noise bandwidth.
fn estimate_snr_2500_db(on_pwr_sum: u64, off_pwr_sum: u64) -> i8 {
    // Mean noise per bin = off_pwr_sum / 36; mean (S + N) per bin = on_pwr_sum / 12 = (3 * on_pwr_sum) / 36.
    let n_36 = off_pwr_sum.max(1);
    let sn_36 = on_pwr_sum.saturating_mul(3);
    // Floor pure signal at 1/16 of per-bin noise (`-12 dB` per-bin = `-33 dB` in 2500 Hz) if noisy fluctuation dips.
    let s_36 = sn_36.saturating_sub(n_36).max((n_36 >> 4).max(1));

    // 10 * log10(s_36 / n_36) = 3.0103 * (log2(s_36) - log2(n_36)).
    // In Q8 (`1/256` log2), `d_q8 * 771 >> 16` = `d_q8 * (3.0117 / 256)` in tenths of a dB (`x10`)? Wait:
    // Let's check exact arithmetic in tenths of a dB (`0.1 dB` units)!
    // `log2_q8` is `256 * log2(x)`.
    // `100 * log10(x) = 30.103 * log2(x) = (log2_q8(x) * 3010 + 12800) / 25600` (in `0.1 dB` units!).
    let diff_q8 = log2_q8(s_36) - log2_q8(n_36);
    let bin_snr_db_x10 = (diff_q8 * 301 + 1280) / 2560;

    // 480-sample (`80 ms`) Hann window has ENBW = `1.5 * 12.5 Hz = 18.75 Hz`.
    // `10 * log10(18.75 / 2500) = -21.25 dB`, plus `+0.55 dB` average half-bin/Hann scalloping loss = `-20.7 dB` (`-207`).
    let snr_2500_x10 = bin_snr_db_x10 - 207;
    let rounded = if snr_2500_x10 >= 0 {
        (snr_2500_x10 + 5) / 10
    } else {
        (snr_2500_x10 - 5) / 10
    };
    rounded.clamp(-24, 20) as i8
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

const fn compute_tone_q_off(spacing_hz: u32) -> [u16; 4] {
    let mut out = [0u16; 4];
    let mut t = 0usize;
    while t < 4 {
        out[t] = ((t as u32 * spacing_hz * FFT_N as u32 + (PROC_RATE_HZ / 4)) / (PROC_RATE_HZ / 2)) as u16;
        t += 1;
    }
    out
}
