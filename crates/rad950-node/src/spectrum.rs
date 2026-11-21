//! Real-time 12 kHz audio spectrum analyzer and waterfall row generator.
//!
//! Uses a 2048-point Hann-windowed radix-2 FFT (`5.859 Hz/bin`) with a 1024-sample
//! hop (`~85.3 ms` = `~11.7 FPS`), covering `351.6 .. 2601.6 Hz` (`384` bins) —
//! sharp enough to visually resolve individual RAD950 4-FSK tones (`15.625 Hz` spacing).

use std::f32::consts::TAU;

pub const WF_FFT_SIZE: usize = 2048;
pub const WF_HOP: usize = 1024;
pub const WF_BIN_START: usize = 60; // 60 * 12000 / 2048 = 351.5625 Hz
pub const WF_BINS: usize = 384; // 384 * 5.859375 = 2250 Hz -> 2601.5625 Hz
pub const WF_ROWS: usize = 180; // ~15.4 s of history at 11.7 FPS
pub const WF_MIN_HZ: f32 = (WF_BIN_START as f32) * 12_000.0 / (WF_FFT_SIZE as f32);
pub const WF_MAX_HZ: f32 = ((WF_BIN_START + WF_BINS) as f32) * 12_000.0 / (WF_FFT_SIZE as f32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RowMarker {
    Normal,
    SlotBoundary,
    Tx,
}

#[derive(Clone, Debug)]
pub struct WaterfallRow {
    /// Power in tenths of dBFS (`-950` = `-95.0 dBFS`, `0` = `0.0 dBFS`).
    pub bins_db10: [i16; WF_BINS],
    pub marker: RowMarker,
}

impl WaterfallRow {
    pub fn tx_marker() -> Self {
        Self {
            bins_db10: [-1200; WF_BINS],
            marker: RowMarker::Tx,
        }
    }
}

pub struct SpectrumAnalyzer {
    window: Vec<f32>,
    ring: Vec<f32>,
    hop_accum: usize,
    re: Vec<f32>,
    im: Vec<f32>,
    bitrev: Vec<usize>,
    tw_re: Vec<f32>,
    tw_im: Vec<f32>,
    next_slot_boundary: bool,
    skip_rx_rows: usize,
}

impl SpectrumAnalyzer {
    pub fn new() -> Self {
        let n = WF_FFT_SIZE;
        let window: Vec<f32> = (0..n)
            .map(|i| 0.5 * (1.0 - (TAU * i as f32 / n as f32).cos()))
            .collect();

        let bits = n.trailing_zeros();
        let bitrev: Vec<usize> = (0..n)
            .map(|i| (i.reverse_bits()) >> (usize::BITS - bits))
            .collect();

        let mut tw_re = Vec::with_capacity(n / 2);
        let mut tw_im = Vec::with_capacity(n / 2);
        for k in 0..(n / 2) {
            let ang = -TAU * (k as f32) / (n as f32);
            tw_re.push(ang.cos());
            tw_im.push(ang.sin());
        }

        Self {
            window,
            ring: vec![0.0; n],
            hop_accum: 0,
            re: vec![0.0; n],
            im: vec![0.0; n],
            bitrev,
            tw_re,
            tw_im,
            next_slot_boundary: false,
            skip_rx_rows: 0,
        }
    }

    /// Tag the next emitted waterfall row with a UTC slot boundary line.
    pub fn mark_slot_boundary(&mut self) {
        self.next_slot_boundary = true;
    }

    /// Clear sliding sample buffer and skip the next `n` emitted RX rows (~85 ms each)
    /// so SDR/relay TX->RX settling transients never flash across the panafall.
    pub fn blank_next_rx_rows(&mut self, n: usize) {
        self.ring.fill(0.0);
        self.hop_accum = 0;
        self.skip_rx_rows = n;
    }

    /// Push arbitrary-length 12 kHz `i16` PCM chunk; invokes `on_row(row, chunk_peak_pct)`
    /// for every completed `WF_HOP` (1024-sample) frame.
    pub fn push_pcm<F>(&mut self, pcm: &[i16], on_row: F)
    where
        F: FnMut(WaterfallRow, f32),
    {
        self.push_pcm_marked(pcm, false, on_row);
    }

    /// Push synthesized 12 kHz TX monitor PCM, tagging emitted rows with `RowMarker::Tx`.
    pub fn push_pcm_tx<F>(&mut self, pcm: &[i16], on_row: F)
    where
        F: FnMut(WaterfallRow, f32),
    {
        self.push_pcm_marked(pcm, true, on_row);
    }

    fn push_pcm_marked<F>(&mut self, pcm: &[i16], is_tx: bool, mut on_row: F)
    where
        F: FnMut(WaterfallRow, f32),
    {
        if pcm.is_empty() {
            return;
        }
        let mut offset = 0usize;
        while offset < pcm.len() {
            let need = WF_HOP - self.hop_accum;
            let take = need.min(pcm.len() - offset);

            // Shift ring left by `take` and append new normalized samples
            self.ring.copy_within(take.., 0);
            let dst_start = WF_FFT_SIZE - take;
            for i in 0..take {
                self.ring[dst_start + i] = pcm[offset + i] as f32 / 32768.0;
            }
            self.hop_accum += take;
            offset += take;

            if self.hop_accum >= WF_HOP {
                self.hop_accum = 0;
                if !is_tx && self.skip_rx_rows > 0 {
                    self.skip_rx_rows -= 1;
                    continue;
                }
                let (mut row, peak_pct) = self.compute_row();
                if is_tx {
                    row.marker = RowMarker::Tx;
                }
                on_row(row, peak_pct);
            }
        }
    }

    fn compute_row(&mut self) -> (WaterfallRow, f32) {
        let n = WF_FFT_SIZE;
        let mut peak = 0.0f32;
        // Measure peak over the most recent hop
        for &s in &self.ring[(n - WF_HOP)..] {
            let a = s.abs();
            if a > peak {
                peak = a;
            }
        }

        // Apply Hann window + bit-reversal permutation
        for i in 0..n {
            let j = self.bitrev[i];
            self.re[j] = self.ring[i] * self.window[i];
            self.im[j] = 0.0;
        }

        // Iterative Cooley-Tukey radix-2 FFT
        let mut len = 2usize;
        while len <= n {
            let half = len / 2;
            let step = n / len;
            let mut base = 0usize;
            while base < n {
                for k in 0..half {
                    let tw_idx = k * step;
                    let wr = self.tw_re[tw_idx];
                    let wi = self.tw_im[tw_idx];

                    let u_idx = base + k;
                    let v_idx = u_idx + half;
                    let vr = self.re[v_idx];
                    let vi = self.im[v_idx];

                    let tr = vr * wr - vi * wi;
                    let ti = vr * wi + vi * wr;

                    let ur = self.re[u_idx];
                    let ui = self.im[u_idx];

                    self.re[u_idx] = ur + tr;
                    self.im[u_idx] = ui + ti;
                    self.re[v_idx] = ur - tr;
                    self.im[v_idx] = ui - ti;
                }
                base += len;
            }
            len <<= 1;
        }

        // Normalize so a full-scale sine wave (amp 1.0) reads ~0 dBFS.
        // Hann window coherent gain = 0.5; single-sided sine magnitude = N/4.
        let norm = (n as f32) * 0.25;
        let inv_norm_sq = 1.0 / (norm * norm);

        let mut bins_db10 = [-1200i16; WF_BINS];
        for b in 0..WF_BINS {
            let idx = WF_BIN_START + b;
            let re = self.re[idx];
            let im = self.im[idx];
            let pwr = (re * re + im * im) * inv_norm_sq;
            let db = if pwr > 1e-12 { 10.0 * pwr.log10() } else { -120.0 };
            bins_db10[b] = (db * 10.0).clamp(-1200.0, 100.0).round() as i16;
        }

        let marker = if self.next_slot_boundary {
            self.next_slot_boundary = false;
            RowMarker::SlotBoundary
        } else {
            RowMarker::Normal
        };

        (WaterfallRow { bins_db10, marker }, (peak * 100.0).min(100.0))
    }
}

impl Default for SpectrumAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fft_resolves_two_tones_accurately() {
        let mut sa = SpectrumAnalyzer::new();
        let mut pcm = Vec::with_capacity(4096);
        for i in 0..4096 {
            let t = i as f32 / 12_000.0;
            let s = 0.5 * (TAU * 1000.0 * t).sin() + 0.25 * (TAU * 2000.0 * t).sin();
            pcm.push((s * 32767.0) as i16);
        }
        let mut rows = Vec::new();
        sa.push_pcm(&pcm, |row, _pk| rows.push(row));
        assert_eq!(rows.len(), 4);

        let last = rows.last().unwrap();
        let bin_1000 = ((1000.0 - WF_MIN_HZ) / (12_000.0 / WF_FFT_SIZE as f32)).round() as usize;
        let bin_2000 = ((2000.0 - WF_MIN_HZ) / (12_000.0 / WF_FFT_SIZE as f32)).round() as usize;
        let bin_1500 = ((1500.0 - WF_MIN_HZ) / (12_000.0 / WF_FFT_SIZE as f32)).round() as usize;

        // 0.5 amp -> ~ -6 dBFS; 0.25 amp -> ~ -12 dBFS; noise floor at 1500 Hz < -70 dBFS
        let db_1000 = last.bins_db10[bin_1000] as f32 / 10.0;
        let db_2000 = last.bins_db10[bin_2000] as f32 / 10.0;
        let db_1500 = last.bins_db10[bin_1500] as f32 / 10.0;

        assert!((db_1000 - (-6.0)).abs() < 1.5, "1000 Hz expected ~-6 dB, got {db_1000}");
        assert!((db_2000 - (-12.0)).abs() < 1.5, "2000 Hz expected ~-12 dB, got {db_2000}");
        assert!(db_1500 < -65.0, "1500 Hz floor expected < -65 dB, got {db_1500}");
    }
}
