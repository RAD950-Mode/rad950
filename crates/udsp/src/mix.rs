//! Mixing and decimation: shift a band of interest to 0 Hz and lower the sample rate, so later stages work on a few
//! hundred complex samples per second instead of thousands of real ones. Integer only.
//!
//! Each input sample is mixed once as it arrives (phase accumulator into the sine table); each output is a plain
//! multiply-accumulate over the delay line with a low-pass filter built at compile time by [`lowpass`]. RAM: the
//! delay line, `L` complex 16-bit samples.

use crate::trig::{ccos, cos_sin, csin};

#[cfg(any(udsp_dsp, feature = "packed"))]
use crate::arm::cdot;
#[cfg(not(any(udsp_dsp, feature = "packed")))]
use crate::arm::cdot_plain as cdot;


/// Low-pass taps, Q15, DC gain 1: Hamming-windowed sinc with cutoff `fc` (fraction of the sample rate).
pub const fn lowpass<const L: usize>(fc: f64) -> [i16; L] {
    let mut h = [0f64; L];
    let mut sum = 0.0;
    let mid = (L - 1) as f64 / 2.0;
    let mut n = 0;
    while n < L {
        let x = n as f64 - mid;
        let s = if x == 0.0 { 2.0 * fc } else { csin(2.0 * core::f64::consts::PI * fc * x) / (core::f64::consts::PI * x) };
        let w = 0.54 - 0.46 * ccos(2.0 * core::f64::consts::PI * n as f64 / (L - 1) as f64);
        h[n] = s * w;
        sum += h[n];
        n += 1;
    }
    let mut out = [0i16; L];
    let mut n = 0;
    while n < L {
        let v = h[n] / sum * 32767.0;
        out[n] = if v >= 0.0 { (v + 0.5) as i16 } else { (v - 0.5) as i16 };
        n += 1;
    }
    out
}

/// Mixer and decimating filter. `L` is the number of taps.
pub struct FrontEnd<const L: usize> {
    /// Mixed (complex) input samples, oldest overwritten first.
    ring: [[i16; 2]; L],
    pos: usize,
    /// Mixer phase of the newest sample (2^32 = one turn).
    phase: u32,
    count: usize,
}

impl<const L: usize> FrontEnd<L> {
    pub const fn new() -> Self {
        FrontEnd { ring: [[0; 2]; L], pos: 0, phase: 0, count: 0 }
    }

    /// Feed one real sample. Every `decim` samples, returns the filtered sample at baseband, the band around the
    /// mixer frequency (`step = f_mix / rate x 2^32`) now centred on 0 Hz. Output has about half the input's scale.
    #[inline]
    pub fn push(&mut self, x: i16, taps: &[i16; L], step: u32, decim: usize) -> Option<(i16, i16)> {
        // Mix: x e^{-i phase}.
        let (c, sn) = cos_sin((self.phase >> 20) as usize);
        self.ring[self.pos] = [((x as i32 * c) >> 15) as i16, (-(x as i32 * sn) >> 15) as i16];
        self.pos = if self.pos + 1 == L { 0 } else { self.pos + 1 };
        self.phase = self.phase.wrapping_add(step);
        self.count += 1;
        if self.count < decim {
            return None;
        }
        self.count = 0;
        // Filter: sum_k ring[n - k] h[k] (the taps are symmetric, so their order does not matter).
        // The delay line is two contiguous runs, oldest first: ring[pos..] then ring[..pos].
        let (newer, older) = self.ring.split_at(self.pos);
        let (t_old, t_new) = taps.split_at(older.len());
        // Two taps per step with SMLAD on an ARMv7E-M target (crate::arm::cdot), one at a time elsewhere; the same
        // 32-bit sums.
        let (r0, i0) = cdot(older, t_old);
        let (r1, i1) = cdot(newer, t_new);
        let (re, im) = (r0.wrapping_add(r1), i0.wrapping_add(i1));
        Some(((re >> 15).clamp(-32768, 32767) as i16, (im >> 15).clamp(-32768, 32767) as i16))
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl<const L: usize> Default for FrontEnd<L> {
    fn default() -> Self {
        Self::new()
    }
}
