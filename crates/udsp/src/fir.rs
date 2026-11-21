//! Decimating FIR filter for real signals: low-pass (taps from [`crate::mix::lowpass`]) and keep every `D`th output.
//! RAM: the delay line, `L` 16-bit samples.

pub struct Decimator<const L: usize> {
    ring: [i16; L],
    pos: usize,
    count: usize,
}

impl<const L: usize> Decimator<L> {
    pub const fn new() -> Self {
        Decimator { ring: [0; L], pos: 0, count: 0 }
    }

    /// Feed one sample; every `decim` samples returns the filtered one.
    #[inline]
    pub fn push(&mut self, x: i16, taps: &[i16; L], decim: usize) -> Option<i16> {
        self.ring[self.pos] = x;
        self.pos = if self.pos + 1 == L { 0 } else { self.pos + 1 };
        self.count += 1;
        if self.count < decim {
            return None;
        }
        self.count = 0;
        let (newer, older) = self.ring.split_at(self.pos);
        let (t_old, t_new) = taps.split_at(older.len());
        let mut acc = 0i32;
        for (&v, &h) in older.iter().zip(t_old) {
            acc += v as i32 * h as i32;
        }
        for (&v, &h) in newer.iter().zip(t_new) {
            acc += v as i32 * h as i32;
        }
        Some((acc >> 15).clamp(-32768, 32767) as i16)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl<const L: usize> Default for Decimator<L> {
    fn default() -> Self {
        Self::new()
    }
}
