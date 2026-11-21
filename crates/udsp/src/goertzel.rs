//! Goertzel filter: the power at one frequency over a block of samples, a few multiply-adds per sample. The usual
//! tool for tone detection (DTMF, CTCSS, selective calling).
//!
//! [`Goertzel::fraction`] scales the power to the share of the block's energy at that frequency, so thresholds do
//! not depend on signal level: a pure on-frequency tone gives about 1.0, noise and other tones much less.

use crate::trig::{cos_sin_phase, phase_step};

#[derive(Clone, Copy, Debug)]
pub struct Goertzel {
    /// 2 cos(2 pi f / rate), Q15.
    coeff: i32,
    s1: i32,
    s2: i32,
}

impl Goertzel {
    /// A filter for `f_mhz` millihertz at `rate_hz` samples per second.
    pub fn new(f_mhz: u32, rate_hz: u32) -> Self {
        let (c, _) = cos_sin_phase(phase_step(f_mhz as u64, rate_hz as u64 * 1000));
        Goertzel { coeff: 2 * c, s1: 0, s2: 0 }
    }

    /// Feed one sample. Inputs should be scaled so a block's sum stays within i32 (block length x 32767 is fine up
    /// to about 65,000 samples).
    #[inline]
    pub fn push(&mut self, x: i16) {
        let s = x as i32 + ((self.coeff as i64 * self.s1 as i64) >> 15) as i32 - self.s2;
        self.s2 = self.s1;
        self.s1 = s;
    }

    /// |X|^2 for the block so far (unnormalised).
    pub fn power(&self) -> u64 {
        let (s1, s2) = (self.s1 as i64, self.s2 as i64);
        let p = s1 * s1 + s2 * s2 - ((self.coeff as i64 * s1) >> 15) * s2;
        p.max(0) as u64
    }

    /// Share of the block's energy at this frequency, Q16 (65536 = all of it), given the block's length and energy
    /// (sum of squares).
    pub fn fraction(&self, n: u32, energy: u64) -> u32 {
        if energy == 0 {
            return 0;
        }
        // A tone of amplitude A: |X|^2 = (N A / 2)^2, energy = N A^2 / 2, so |X|^2 * 2 / (N energy) = 1.
        let num = self.power() as u128 * 2 * 65536;
        (num / (n as u128 * energy as u128)).min(u32::MAX as u128) as u32
    }

    /// Start a new block.
    pub fn reset(&mut self) {
        self.s1 = 0;
        self.s2 = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `n` samples of an `f` hertz tone at 8000 samples a second (quarter scale, from the crate's own table) into
    /// `g`; returns their energy.
    fn tone(g: &mut Goertzel, f: u32, n: u32) -> u64 {
        let mut energy = 0u64;
        for i in 0..n {
            let (_, s) = cos_sin_phase(phase_step((f as u64 * i as u64 % 8000) * 1000, 8_000_000));
            let x = (s / 4) as i16;
            g.push(x);
            energy += (x as i64 * x as i64) as u64;
        }
        energy
    }

    /// DTMF row 697 Hz against column 1209 Hz over a 205-sample block, with a reset between blocks.
    #[test]
    fn dtmf_tones_and_reset() {
        let n = 205;
        let mut g = Goertzel::new(697_000, 8000);
        let e = tone(&mut g, 697, n);
        let on = g.fraction(n, e);
        assert!((on as i64 - 65536).abs() < 65536 / 20, "on-frequency share {on}");
        g.reset();
        assert_eq!(g.power(), 0);
        let e = tone(&mut g, 1209, n);
        let off = g.fraction(n, e);
        assert!(off < 65536 / 100, "off-frequency share {off}");
        // After a reset the first block is repeated exactly.
        g.reset();
        let e = tone(&mut g, 697, n);
        assert_eq!(g.fraction(n, e), on);
    }

    #[test]
    fn silence_has_no_share() {
        let mut g = Goertzel::new(1_000_000, 8000);
        for _ in 0..100 {
            g.push(0);
        }
        assert_eq!(g.power(), 0);
        assert_eq!(g.fraction(100, 0), 0);
    }
}
