//! Clean-room `#![no_std]` fixed-point DSP primitives for RAD950.
//!
//! Designed specifically for RAD950's 24 kHz → 6 kHz decimation, 512-point real FFT,
//! 4-FSK phase-continuous modulation, and integer log-power scoring:
//! - Quarter-wave `const` sine LUT (2 KB instead of 8 KB) with exact 4-quadrant symmetry
//! - Symmetric-folded linear-phase FIR [`Decimator`] (half the multiplications of a naive FIR)
//! - Block-floating-point radix-2 DIT [`cfft`] + half-size real-FFT [`real_power`] unpacker
//! - Branchless `clz` + 64-entry interpolated [`log2_q8`] (<0.01 dB error)

use core::f64::consts::PI;

/// Compile-time `cos(x)` via range reduction + minimax/Taylor polynomial (exact to ~1e-15).
pub const fn ccos(mut x: f64) -> f64 {
    const TAU: f64 = 2.0 * PI;
    // Wrap into [-PI, PI]
    let k = (x / TAU) as i64;
    x -= (k as f64) * TAU;
    if x > PI {
        x -= TAU;
    } else if x < -PI {
        x += TAU;
    }
    if x < 0.0 {
        x = -x;
    }
    let sign = if x > PI * 0.5 {
        x = PI - x;
        -1.0
    } else {
        1.0
    };
    let x2 = x * x;
    // 10-term Taylor series on [0, PI/2] gives full IEEE-754 f64 precision
    let mut term = 1.0;
    let mut sum = 1.0;
    let mut n = 1usize;
    while n <= 10 {
        term = -term * x2 / ((2 * n - 1) * (2 * n)) as f64;
        sum += term;
        n += 1;
    }
    sign * sum
}

/// Compile-time `sin(x)` derived from [`ccos`].
pub const fn csin(x: f64) -> f64 {
    ccos(x - PI * 0.5)
}

/// Quarter-wave sine table (`0..=1024` covering `0..=PI/2` of a 4096-step circle) in Q15.
const QTR: usize = 1024;
const SIN_QTR: [i16; QTR + 1] = {
    let mut t = [0i16; QTR + 1];
    let mut i = 0usize;
    while i <= QTR {
        let v = csin((i as f64) * (PI * 0.5) / (QTR as f64)) * 32767.0;
        t[i] = (v + 0.5) as i16;
        i += 1;
    }
    t
};

/// Fast sine lookup for a 12-bit angle `k` (`0..4096` = `0..2π`), Q15 (`-32767..=+32767`).
#[inline]
fn sin_4096(k: u32) -> i16 {
    let idx = (k & 4095) as usize;
    match idx >> 10 {
        0 => SIN_QTR[idx],
        1 => SIN_QTR[2048 - idx],
        2 => -SIN_QTR[idx - 2048],
        _ => -SIN_QTR[4096 - idx],
    }
}

/// `(cos, sin)` in Q15 (`-32767..=+32767`) for a 12-bit angle `k` (`0..4096` = `0..2π`).
#[inline]
pub fn cos_sin(k: usize) -> (i32, i32) {
    (sin_4096((k as u32).wrapping_add(1024)) as i32, sin_4096(k as u32) as i32)
}

/// `(cos, sin)` in Q15 (`-32767..=+32767`) for a 32-bit phase accumulator (`0..2^32` = `0..2π`),
/// using linear interpolation between adjacent 12-bit table entries.
#[inline]
pub fn cos_sin_phase(phase: u32) -> (i32, i32) {
    let idx = (phase >> 20) as usize;
    let frac = ((phase >> 12) & 0xFF) as i32; // 8-bit fractional blend
    let (c0, s0) = cos_sin(idx);
    let (c1, s1) = cos_sin(idx.wrapping_add(1));
    let c = c0 + (((c1 - c0) * frac + 128) >> 8);
    let s = s0 + (((s1 - s0) * frac + 128) >> 8);
    (c, s)
}

/// 32-bit phase increment per sample for frequency `freq_num` at sample rate `rate_den`
/// (both in the same units, e.g. millihertz).
#[inline]
pub const fn phase_step(freq_num: u64, rate_den: u64) -> u32 {
    if rate_den == 0 {
        return 0;
    }
    (((freq_num as u128) << 32) / (rate_den as u128)) as u32
}

/// Windowed-sinc linear-phase lowpass FIR tap designer in Q15 (Blackman window, unity DC gain).
/// `cutoff` is normalized frequency `fc / fs` (`0.0 < cutoff < 0.5`).
pub const fn lowpass<const N: usize>(cutoff: f64) -> [i16; N] {
    let mut f = [0.0f64; N];
    let mid = (N - 1) as f64 * 0.5;
    let mut sum = 0.0;
    let mut i = 0usize;
    while i < N {
        let d = i as f64 - mid;
        let sinc = if d == 0.0 {
            2.0 * cutoff
        } else {
            csin(2.0 * PI * cutoff * d) / (PI * d)
        };
        let a = 2.0 * PI * (i as f64) / ((N - 1) as f64);
        let win = 0.42 - 0.5 * ccos(a) + 0.08 * ccos(2.0 * a);
        f[i] = sinc * win;
        sum += f[i];
        i += 1;
    }
    let mut taps = [0i16; N];
    let mut j = 0usize;
    while j < N {
        let q = (f[j] / sum) * 32767.0;
        taps[j] = if q >= 0.0 { (q + 0.5) as i16 } else { (q - 0.5) as i16 };
        j += 1;
    }
    taps
}

/// Symmetric-folded FIR decimator (`N` taps, exploiting `taps[k] == taps[N-1-k]` to halve MACs).
pub struct Decimator<const N: usize> {
    buf: [i16; N],
    pos: usize,
    phase: usize,
}

impl<const N: usize> Default for Decimator<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Decimator<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            pos: 0,
            phase: 0,
        }
    }

    pub fn reset(&mut self) {
        self.buf = [0; N];
        self.pos = 0;
        self.phase = 0;
    }

    /// Push one input sample; emits one filtered output sample every `factor` inputs.
    #[inline]
    pub fn push(&mut self, x: i16, taps: &[i16; N], factor: usize) -> Option<i16> {
        self.buf[self.pos] = x;
        self.pos += 1;
        if self.pos == N {
            self.pos = 0;
        }
        self.phase += 1;
        if self.phase < factor {
            return None;
        }
        self.phase = 0;

        // Fold symmetric pairs around the newest sample at `pos - 1`:
        // newest is index 0 of delay line, oldest is index N - 1.
        let mut acc: i64 = 0;
        let half = N / 2;
        let mut idx_lo = if self.pos == 0 { N - 1 } else { self.pos - 1 };
        let mut idx_hi = self.pos; // oldest
        let mut k = 0usize;
        while k < half {
            let pair = self.buf[idx_lo] as i32 + self.buf[idx_hi] as i32;
            acc += (pair as i64) * (taps[k] as i64);
            idx_lo = if idx_lo == 0 { N - 1 } else { idx_lo - 1 };
            idx_hi = if idx_hi + 1 == N { 0 } else { idx_hi + 1 };
            k += 1;
        }
        if N % 2 == 1 {
            acc += (self.buf[idx_lo] as i64) * (taps[half] as i64);
        }
        let y = ((acc + 16_384) >> 15).clamp(-32768, 32767);
        Some(y as i16)
    }
}

/// In-place block-floating-point radix-2 DIT complex FFT on interleaved `[re0, im0, re1, im1, ...]`
/// of length `2 * m` (`m` must be a power of 2, `<= 2048`).
///
/// Returns the total number of right-shifts (`shift`) applied across stages to prevent `i16` overflow.
pub fn cfft(z: &mut [i16], m: usize) -> u8 {
    debug_assert!(m.is_power_of_two() && m <= 2048 && z.len() >= 2 * m);
    let log2_m = m.trailing_zeros();

    // 1. Bit-reversal permutation
    for i in 0..m {
        let j = (i.reverse_bits()) >> (usize::BITS - log2_m);
        if j > i {
            z.swap(2 * i, 2 * j);
            z.swap(2 * i + 1, 2 * j + 1);
        }
    }

    // 2. Radix-2 DIT stages with adaptive block-floating-point scaling
    let mut shift: u8 = 0;
    let mut half = 1usize;
    while half < m {
        let step = half << 1;
        // Scale down by 2 if any element could overflow a butterfly sum (`|x| > 16000`)
        let mut max_abs = 0i32;
        for &v in &z[..2 * m] {
            let a = (v as i32).abs();
            if a > max_abs {
                max_abs = a;
            }
        }
        let scale = if max_abs > 16_000 {
            shift = shift.saturating_add(1);
            1
        } else {
            0
        };

        let ang_step = 4096 / step;
        let mut base = 0usize;
        while base < m {
            let mut k = 0usize;
            let mut ang = 0usize;
            while k < half {
                let (wr, wi_pos) = cos_sin(ang);
                let wi = -wi_pos; // W_N^k = cos - j*sin

                let ia = 2 * (base + k);
                let ib = 2 * (base + k + half);
                let ar = (z[ia] as i32) >> scale;
                let ai = (z[ia + 1] as i32) >> scale;
                let br = (z[ib] as i32) >> scale;
                let bi = (z[ib + 1] as i32) >> scale;

                let tr = (br * wr - bi * wi + 16_384) >> 15;
                let ti = (br * wi + bi * wr + 16_384) >> 15;

                z[ia] = (ar + tr).clamp(-32768, 32767) as i16;
                z[ia + 1] = (ai + ti).clamp(-32768, 32767) as i16;
                z[ib] = (ar - tr).clamp(-32768, 32767) as i16;
                z[ib + 1] = (ai - ti).clamp(-32768, 32767) as i16;

                k += 1;
                ang += ang_step;
            }
            base += step;
        }
        half = step;
    }
    shift
}

/// Unpack real-input FFT bin power `|X[k]|^2` from a half-size complex FFT `z` of length `2 * m`
/// (`N = 2 * m` real samples packed as `(even, odd)` pairs before calling [`cfft`]).
///
/// Calls `cb(bin, power)` for `count` bins starting at `first_bin` with stride `step`.
pub fn real_power<F: FnMut(usize, u64)>(
    z: &[i16],
    m: usize,
    first_bin: usize,
    step: usize,
    count: usize,
    mut cb: F,
) {
    debug_assert!(m.is_power_of_two() && z.len() >= 2 * m);
    let ang_per_bin = 4096 / (2 * m);
    let mut k = first_bin;
    for _ in 0..count {
        let km = (m - k) & (m - 1);
        let zr = z[2 * k] as i32;
        let zi = z[2 * k + 1] as i32;
        let cr = z[2 * km] as i32;
        let ci = z[2 * km + 1] as i32;

        // E = (Z[k] + conj(Z[m - k])) / 2
        let er = (zr + cr) >> 1;
        let ei = (zi - ci) >> 1;
        // O = -j * (Z[k] - conj(Z[m - k])) / 2
        let or_ = (zi + ci) >> 1;
        let oi = (cr - zr) >> 1;

        // W_{2m}^k = cos(2*pi*k/(2m)) - j*sin(2*pi*k/(2m))
        let (wr, wi_pos) = cos_sin(k * ang_per_bin);
        let wi = -wi_pos;

        let tr = (or_ * wr - oi * wi + 16_384) >> 15;
        let ti = (or_ * wi + oi * wr + 16_384) >> 15;

        let xr = er + tr;
        let xi = ei + ti;
        let pwr = ((xr as i64) * (xr as i64) + (xi as i64) * (xi as i64)) as u64;
        cb(k, pwr);
        k += step;
    }
}

/// 64-entry `const` LUT of `256 * log2(1 + i/64)` for fast interpolated [`log2_q8`].
const LOG2_LUT: [u16; 65] = {
    let mut t = [0u16; 65];
    let mut i = 0usize;
    while i <= 64 {
        // Compute ln(1 + i/64) via atanh series: ln(y) = 2 * sum_{k=0..} u^(2k+1)/(2k+1), u = (y-1)/(y+1)
        let y = 1.0 + (i as f64) / 64.0;
        let u = (y - 1.0) / (y + 1.0);
        let u2 = u * u;
        let mut p = u;
        let mut ln_y = 0.0;
        let mut k = 0usize;
        while k < 12 {
            ln_y += p / ((2 * k + 1) as f64);
            p *= u2;
            k += 1;
        }
        ln_y *= 2.0;
        let l2 = ln_y / core::f64::consts::LN_2;
        t[i] = (l2 * 256.0 + 0.5) as u16;
        i += 1;
    }
    t
};

/// Fast fixed-point `256 * log2(x)` (`Q8` fixed-point, <0.01 dB error for all `x >= 1`).
#[inline]
pub fn log2_q8(x: u64) -> i32 {
    if x <= 1 {
        return 0;
    }
    let exp = 63 - x.leading_zeros();
    // Normalize mantissa so top bit is at bit 63: bits 57..62 = 6-bit LUT index, bits 49..56 = 8-bit frac
    let norm = x << (63 - exp);
    let idx = ((norm >> 57) & 63) as usize;
    let frac = ((norm >> 49) & 255) as i32;
    let y0 = LOG2_LUT[idx] as i32;
    let y1 = LOG2_LUT[idx + 1] as i32;
    let mant = y0 + (((y1 - y0) * frac + 128) >> 8);
    ((exp as i32) << 8) + mant
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trig_and_log2_accuracy() {
        assert_eq!(cos_sin(0), (32767, 0));
        assert_eq!(cos_sin(1024), (0, 32767));
        assert_eq!(cos_sin(2048), (-32767, 0));
        assert_eq!(cos_sin(3072), (0, -32767));

        for &v in &[1u64, 2, 3, 10, 100, 1024, 65535, 1_000_000, u32::MAX as u64, 1u64 << 45] {
            let got = log2_q8(v) as f64 / 256.0;
            let want = (v as f64).log2();
            assert!((got - want).abs() < 0.005, "log2_q8({v}): got {got} want {want}");
        }
    }

    #[test]
    fn real_fft_detects_single_tone() {
        const N: usize = 512;
        const M: usize = N / 2;
        let mut z = [0i16; N];
        let target_bin = 64usize;
        for (n, s) in z.iter_mut().enumerate() {
            let ang = (n * target_bin * 4096) / N;
            *s = (cos_sin(ang).0 / 2) as i16;
        }
        let _shift = cfft(&mut z, M);
        let mut best_bin = 0usize;
        let mut best_pwr = 0u64;
        real_power(&z, M, 1, 1, M - 2, |b, p| {
            if p > best_pwr {
                best_pwr = p;
                best_bin = b;
            }
        });
        assert_eq!(best_bin, target_bin);
        assert!(best_pwr > 100_000);
    }
}
