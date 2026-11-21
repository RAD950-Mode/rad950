//! RAD950 physical layer: `6 + 6 + 6` 4-ary sync placement, stride-37 bit interleaver, Gray tone mapping,
//! and reproducible integer phase-continuous TX modulator (`10 ms` raised-cosine ramp + `70 ms` steady tone).

use crate::frame::Frame;
use crate::ldpc::{self, N_BITS};
use crate::dsp::{cos_sin, cos_sin_phase, phase_step};

pub const N_SYMBOLS: usize = 110;
pub const N_SYNC_SYMS: usize = 18;
pub const N_DATA_SYMS: usize = N_SYMBOLS - N_SYNC_SYMS; // 92
pub const SYMBOL_MS: u32 = 80;
pub const RAMP_MS: u32 = 10;
pub const SUBSTEPS_RAMP: usize = 8; // 8 x 1.25 ms = 10 ms ramp
pub const SUBSTEPS_SYM: usize = 64; // 64 x 1.25 ms = 80 ms symbol
pub const DEFAULT_SPACING_HZ: u32 = 50;

/// Optimal balanced non-repeating 4-ary sync sequences (`S0 @ 0..6`, `S1 @ 18..24`, `S2 @ 104..110`).
pub static S0: [u8; 6] = [0, 3, 0, 1, 3, 2];
pub static S1: [u8; 6] = [3, 0, 3, 2, 0, 1];
pub static S2: [u8; 6] = [3, 1, 0, 2, 3, 0];

/// Subchannel lowest-tone offsets (`Hz` above USB dial) across `600..2,400 Hz` (`225 Hz` steps = 200 Hz span + 25 Hz guard).
pub static SUBCHANNELS_HZ: [u16; 8] = [600, 825, 1050, 1275, 1500, 1725, 1950, 2175];

/// Gray map from dibit `(b0 << 1) | b1` to tone index `0..=3`:
/// - tone `0`: `(0, 0)`
/// - tone `1`: `(0, 1)`
/// - tone `2`: `(1, 1)`
/// - tone `3`: `(1, 0)`
pub static GRAY_ENCODE: [u8; 4] = [0, 1, 3, 2];

/// Stride-37 bit interleaver (`gcd(37, 184) = 1`), spreading adjacent codeword bits ~3 seconds apart.
pub const INTERLEAVE_STRIDE: usize = 37;

#[inline]
pub const fn interleave_idx(code_bit: usize) -> usize {
    (code_bit * INTERLEAVE_STRIDE) % N_BITS
}

/// Map data symbol index `d in 0..92` to frame symbol index `0..110`.
#[inline]
pub const fn data_sym_pos(d: usize) -> usize {
    if d < 12 {
        6 + d
    } else {
        24 + (d - 12)
    }
}

/// Encode a [`Frame`] all the way to its 110 transmitted tone indices (`0..=3`).
pub fn encode_tones(frame: Frame) -> [u8; N_SYMBOLS] {
    let bits78 = frame.to_bits78();
    let cw = ldpc::encode(&bits78);

    let mut inter = [0u8; N_BITS];
    for (i, &b) in cw.iter().enumerate() {
        inter[interleave_idx(i)] = b;
    }

    let mut tones = [0u8; N_SYMBOLS];
    tones[0..6].copy_from_slice(&S0);
    tones[18..24].copy_from_slice(&S1);
    tones[104..110].copy_from_slice(&S2);

    for d in 0..N_DATA_SYMS {
        let b0 = inter[2 * d] & 1;
        let b1 = inter[2 * d + 1] & 1;
        let dibit = ((b0 << 1) | b1) as usize;
        tones[data_sym_pos(d)] = GRAY_ENCODE[dibit];
    }
    tones
}

/// Raised-cosine blend weight in `0..=32767` for fractional position `num / den` (`0 <= num <= den`).
#[inline]
pub fn raised_cosine_q15(num: usize, den: usize) -> i32 {
    if num == 0 || den == 0 {
        return 0;
    }
    if num >= den {
        return 32_767;
    }
    // Half turn = 2048 steps in udsp's 4096-entry circle: (1 - cos(pi * num / den)) / 2 in Q15.
    let k = (num * 2048 + den / 2) / den;
    let c = cos_sin(k).0; // +32767 at k=0, -32767 at k=2048
    (32_767 - c) >> 1
}

/// BK4829 `REG_70` tone-1 gain (`0..=127`) at `1.25 ms` sub-step `sub in 0..64` within symbol `sym in 0..110`,
/// implementing the exact §2.3 `10 ms` (8 sub-steps) raised-cosine transition + `70 ms` (56 sub-steps) steady hold.
pub fn reg70_gain_at(tones: &[u8; N_SYMBOLS], sym: usize, sub: usize, tone_gains: [u8; 4]) -> u8 {
    let cur = tone_gains[(tones[sym] & 3) as usize] as i32;
    if sym == 0 || sub >= SUBSTEPS_RAMP {
        return cur as u8;
    }
    let prev = tone_gains[(tones[sym - 1] & 3) as usize] as i32;
    if prev == cur {
        return cur as u8;
    }
    let w = raised_cosine_q15(sub, SUBSTEPS_RAMP);
    (prev + (((cur - prev) * w + 16_384) >> 15)).clamp(0, 127) as u8
}

/// Reproducible, integer-only, phase-continuous 4-FSK modulator implementing §2.3 (`10 ms` raised-cosine ramp +
/// `70 ms` steady tone per `80 ms` symbol).
pub struct Modulator {
    tones: [u8; N_SYMBOLS],
    rate_hz: u32,
    sym_samples: u32,
    ramp_samples: u32,
    f_off_mhz: u64,
    spacing_mhz: u64,
    /// Optional linear carrier drift across the 110-symbol burst in millihertz (for swept-carrier receiver tests).
    total_drift_mhz: i64,
    amp: i16,
    phase: u32,
    sym: usize,
    sample_in_sym: u32,
}

impl Modulator {
    /// Create a modulator for `tones` at `rate_hz` (`24_000` or `48_000`), lowest-tone offset `f_off_hz`
    /// (e.g. `1050`), tone spacing `spacing_hz` (e.g. `50`), and peak amplitude `amp` (`<= 32767`).
    pub const fn new(tones: [u8; N_SYMBOLS], rate_hz: u32, f_off_hz: u32, spacing_hz: u32, amp: i16) -> Self {
        Self::with_drift_mhz(tones, rate_hz, f_off_hz, spacing_hz, 0, amp)
    }

    /// Create a modulator with linear carrier frequency sweep `total_drift_mhz` (in mHz, e.g. `+10_000` = `+10 Hz`
    /// from symbol 0 to symbol 110) for receiver drift-tracking verification.
    pub const fn with_drift_mhz(
        tones: [u8; N_SYMBOLS],
        rate_hz: u32,
        f_off_hz: u32,
        spacing_hz: u32,
        total_drift_mhz: i64,
        amp: i16,
    ) -> Self {
        let sym_samples = (rate_hz * SYMBOL_MS) / 1000;
        let ramp_samples = (rate_hz * RAMP_MS) / 1000;
        Self {
            tones,
            rate_hz,
            sym_samples,
            ramp_samples,
            f_off_mhz: f_off_hz as u64 * 1000,
            spacing_mhz: spacing_hz as u64 * 1000,
            total_drift_mhz,
            amp,
            phase: 0,
            sym: 0,
            sample_in_sym: 0,
        }
    }

    pub const fn total_samples(&self) -> usize {
        self.sym_samples as usize * N_SYMBOLS
    }

    pub const fn is_done(&self) -> bool {
        self.sym >= N_SYMBOLS
    }

    /// Instantaneous frequency in millihertz at the current sample.
    #[inline]
    pub fn current_freq_mhz(&self) -> u64 {
        if self.sym >= N_SYMBOLS {
            return self.f_off_mhz;
        }
        let total_n = self.sym_samples as i64 * N_SYMBOLS as i64;
        let cur_n = self.sym as i64 * self.sym_samples as i64 + self.sample_in_sym as i64;
        let drift = if total_n > 0 { (self.total_drift_mhz * cur_n) / total_n } else { 0 };
        let f_base = (self.f_off_mhz as i64 + drift).max(0) as u64;

        let t_cur = (self.tones[self.sym] & 3) as i64;
        if self.sym == 0 || self.sample_in_sym >= self.ramp_samples {
            return f_base + (t_cur as u64) * self.spacing_mhz;
        }
        let t_prev = (self.tones[self.sym - 1] & 3) as i64;
        if t_prev == t_cur {
            return f_base + (t_cur as u64) * self.spacing_mhz;
        }
        let w = raised_cosine_q15(self.sample_in_sym as usize, self.ramp_samples as usize) as i64;
        let base_prev = f_base as i64 + t_prev * self.spacing_mhz as i64;
        let df = (t_cur - t_prev) * self.spacing_mhz as i64;
        (base_prev + ((df * w + 16_384) >> 15)).max(0) as u64
    }

    /// Advance one sample, returning complex `(I, Q)` scaled by `amp`, or `None` after all 110 symbols finish.
    #[inline]
    pub fn next_iq(&mut self) -> Option<(i16, i16)> {
        if self.sym >= N_SYMBOLS {
            return None;
        }
        let f_mhz = self.current_freq_mhz();
        let (c, s) = cos_sin_phase(self.phase);
        let step = phase_step(f_mhz, self.rate_hz as u64 * 1000);
        self.phase = self.phase.wrapping_add(step);

        self.sample_in_sym += 1;
        if self.sample_in_sym >= self.sym_samples {
            self.sample_in_sym = 0;
            self.sym += 1;
        }

        let i_out = ((c * self.amp as i32 + 16_384) >> 15).clamp(-32_767, 32_767) as i16;
        let q_out = ((s * self.amp as i32 + 16_384) >> 15).clamp(-32_767, 32_767) as i16;
        Some((i_out, q_out))
    }

    /// Advance one real audio PCM sample (`I` channel).
    #[inline]
    pub fn next_sample(&mut self) -> Option<i16> {
        self.next_iq().map(|(i, _)| i)
    }
}
