//! One quarter-wave sine table, built at compile time (2 KB of flash), serving every FFT twiddle, window and
//! oscillator in the family; and `const` sin/cos for building other tables at compile time.

/// sin(2 pi k / 4096) for k = 0..=1024, Q15. Serves any FFT size that divides 4096.
pub static SIN: [i16; 1025] = sin_table();

const fn sin_table() -> [i16; 1025] {
    let mut t = [0i16; 1025];
    let mut k = 0;
    while k <= 1024 {
        let x = 2.0 * core::f64::consts::PI * k as f64 / 4096.0;
        // Taylor series, plenty for x <= pi/2 at 16 bits.
        let mut s = 0.0;
        let mut term = x;
        let mut n = 1;
        while n < 30 {
            s += term;
            term = -term * x * x / ((n + 1) as f64 * (n + 2) as f64);
            n += 2;
        }
        let v = s * 32767.0 + 0.5;
        t[k] = if v >= 32767.0 { 32767 } else { v as i16 };
        k += 1;
    }
    t
}

/// (cos, sin) of 2 pi k / 4096, Q15, any k.
// Always inlined: it is in every oscillator's per-sample path, and size-optimised builds otherwise call it.
#[inline(always)]
pub fn cos_sin(k: usize) -> (i32, i32) {
    let k = k & 4095;
    let q = k / 1024;
    let r = k % 1024;
    let (s, c) = (SIN[r] as i32, SIN[1024 - r] as i32);
    match q {
        0 => (c, s),
        1 => (-s, c),
        2 => (-c, -s),
        _ => (s, -c),
    }
}


/// sin for const evaluation: reduce to [-pi, pi], Taylor series.
pub const fn csin(x: f64) -> f64 {
    let two_pi = 2.0 * core::f64::consts::PI;
    let mut r = x - two_pi * ((x / two_pi) as i64 as f64);
    if r > core::f64::consts::PI {
        r -= two_pi;
    } else if r < -core::f64::consts::PI {
        r += two_pi;
    }
    let mut s = 0.0;
    let mut term = r;
    let mut k = 1;
    while k < 40 {
        s += term;
        term = -term * r * r / ((k + 1) as f64 * (k + 2) as f64);
        k += 2;
    }
    s
}

pub const fn ccos(x: f64) -> f64 {
    csin(x + core::f64::consts::FRAC_PI_2)
}


/// (cos, sin) of a phase given as a fraction of a turn in 2^32 units, Q15, interpolated between table entries
/// (error under 2e-5): oscillators and filter coefficients at any frequency and sample rate.
#[inline]
pub fn cos_sin_phase(phase: u32) -> (i32, i32) {
    let k = (phase >> 20) as usize; // 4096 table steps per turn
    let frac = ((phase >> 4) & 0xFFFF) as i32; // position between entries, Q16
    let (c0, s0) = cos_sin(k);
    let (c1, s1) = cos_sin(k + 1);
    (c0 + (((c1 - c0) * frac) >> 16), s0 + (((s1 - s0) * frac) >> 16))
}

/// Phase step (2^32 per turn) of a frequency `f_mhz` (millihertz) at `rate_mhz` (millihertz) samples per second.
#[inline]
pub const fn phase_step(f_mhz: u64, rate_mhz: u64) -> u32 {
    ((f_mhz << 32) / rate_mhz) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The quarter-wave table builder run again at run time gives the flash table, within half a step of the true
    /// sine, from 0 to full scale.
    #[test]
    fn table_matches_its_builder() {
        let t = sin_table();
        assert_eq!(t, SIN);
        assert_eq!((t[0], t[1024]), (0, 32767));
        assert_eq!(t[512], 23170); // sin(pi/4) = 0.70711
        for k in 0..=1024 {
            let want = (2.0 * core::f64::consts::PI * k as f64 / 4096.0).sin() * 32767.0;
            assert!((t[k] as f64 - want).abs() <= 0.5, "k {k}: {} vs {want}", t[k]);
        }
    }

    #[test]
    fn quadrants_and_interpolation() {
        assert_eq!(cos_sin(0), (32767, 0));
        assert_eq!(cos_sin(1024), (0, 32767));
        assert_eq!(cos_sin(2048), (-32767, 0));
        assert_eq!(cos_sin(3072), (0, -32767));
        assert_eq!(cos_sin(4096 + 1024), cos_sin(1024), "any k, modulo a turn");
        // Half way between two table steps: the mean of the two.
        let (c0, s0) = cos_sin(100);
        let (c1, s1) = cos_sin(101);
        let (c, s) = cos_sin_phase((100 << 20) + (1 << 19));
        assert!((c - (c0 + c1) / 2).abs() <= 1 && (s - (s0 + s1) / 2).abs() <= 1);
        // 1 kHz at 8 kHz is an eighth of a turn.
        assert_eq!(phase_step(1_000_000, 8_000_000), 1 << 29);
    }
}
