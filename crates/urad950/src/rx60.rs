//! `RAD950-60` 1-minute weak-signal streaming receiver (`5 × 10 s` v1 bursts in `:00..:49`,
//! `:50..:59` PA cooldown + decode turnaround).
//!
//! Fits strictly inside a `13 KiB` embedded decoder slot (`13,312` bytes, zero heap, `no_std`)
//! by folding all five 10-second sub-bursts into a single 5-bit-per-cell `116 × 150` spectrogram
//! (`10,875` bytes) using 70 ms steady-hold rectangular FFTs and integer `log2_q8`
//! passband-normalized excess power.

use crate::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use crate::frame::Frame;
use crate::ldpc::{self, BpScratch, N_BITS};
use crate::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS, S0, S1, S2, SUBCHANNELS_HZ};
use crate::rx::{Decoded, FFT_M, FFT_N, FIRST_BIN, MAX_DECODED, N_BINS, SYM_SAMPLES};

/// Number of 80 ms symbols folded per 10 s sub-burst (`start_sym 0..=6` + `110` burst symbols = `116`).
/// Fold origin is delayed by [`FOLD_ORIGIN_SYMS`] (`160 ms`), centering nominal `+400 ms` at `start_sym = 3`
/// and covering `[-240 ms, +240 ms]` (`[-160 ms, +240 ms]` for compact) across the `±150 ms` GPS TX gate.
pub const FOLD_SYMS: usize = 116;
const FOLD_ORIGIN_SYMS: usize = 2; // 160 ms delay -> start_sym=3 is nominal +400 ms; ends at 9.44 s (< 10 s)
const NOMINAL_FOLD_START_SYM: i16 = 5 - FOLD_ORIGIN_SYMS as i16; // 3
const HALF_BIN_Q4: usize = 8;
const TONE_SPACING_Q4: usize = 68;
const FOLD_CELLS: usize = FOLD_SYMS * N_BINS; // 17,400
const FOLD_BYTES: usize = (FOLD_CELLS * 5).div_ceil(8); // 10,875 bytes
/// Packed 4-bit half-symbol (`+40 ms`) folded spectrogram (`116 * 150 / 2 = 8,700` bytes).
pub const HALF_FOLD_BYTES: usize = (FOLD_CELLS * 4).div_ceil(8); // 8,700 bytes
const RAMP_SAMPLES_6K: usize = 60; // 10 ms @ 6,000 Hz (skip ramp, integrate 70 ms steady hold)
const COMPACT_CHANNELS: usize = SUBCHANNELS_HZ.len();
const COMPACT_FIRST_SYM: usize = 1 + data_sym_pos(0);
const COMPACT_LAST_SYM: usize = FOLD_SYMS - crate::phy::N_SYMBOLS + data_sym_pos(N_DATA_SYMS - 1);
const COMPACT_SYMS: usize = COMPACT_LAST_SYM - COMPACT_FIRST_SYM + 1;
/// Four three-bit enhancement levels for every published channel and folded data symbol.
pub const COMPACT_SECONDARY_BYTES: usize = (COMPACT_CHANNELS * COMPACT_SYMS * 12).div_ceil(8);

static NOMINAL_HALF_BINS: [usize; COMPACT_CHANNELS] = {
    let mut bins = [0; COMPACT_CHANNELS];
    let mut channel = 0;
    while channel < COMPACT_CHANNELS {
        bins[channel] =
            (SUBCHANNELS_HZ[channel] as usize * (FFT_N * 2) + 3_000) / 6_000 - FIRST_BIN * 2;
        assert!(channel == 0 || bins[channel - 1] < bins[channel]);
        assert!((bins[channel] * 8 + 3 * TONE_SPACING_Q4) / 16 + 1 < N_BINS);
        channel += 1;
    }
    bins
};

/// Decimation low-pass filter (`24 kHz -> 6 kHz`, identical to `urad950::rx`).
static LP_24K_TO_6K: [i16; 31] = lowpass::<31>(2_550.0 / 24_000.0);

/// Streaming `RAD950-60` receiver.
/// - `Receiver60` (`COMPACT_BYTES = 0, HALF_BYTES = 0`): single-phase 5-bit fold for desktop passes.
/// - `CompactReceiver60` (`COMPACT_SECONDARY_BYTES, HALF_FOLD_BYTES`): Layout B handheld receiver
///   combining a 5-bit `0 ms` fold, 1,236 B compact table, and 4-bit `+40 ms` half-symbol fold
///   (`21,864 B` total) to eliminate the half-symbol clock-skew dead zone.
pub struct MinuteReceiver<const COMPACT_BYTES: usize, const HALF_BYTES: usize> {
    decim: Decimator<31>,
    pcm_ring: [i16; SYM_SAMPLES],
    pcm_pos: u16,
    decim_count: u32,
    subslot: u8,
    wf_col: [u8; 12],
    wf_ready: bool,
    percentile_span: bool,
    /// When true (and `HALF_BYTES == 0`), evaluate each 80 ms symbol window shifted by +40 ms
    /// (half-symbol phase) for desktop dual-phase passes.
    phase_half: bool,
    /// Packed 5-bit unsigned folded spectrogram (`FOLD_SYMS * N_BINS` cells, `0..=31`).
    fold: [u8; FOLD_BYTES],
    /// Four three-bit enhancement levels for each published channel and folded data symbol.
    /// The 1,236-byte table gives the secondary decoder stronger local contrast without
    /// retaining a second complete spectrum or running a second FFT.
    compact: [u8; COMPACT_BYTES],
    /// Packed 4-bit unsigned `+40 ms` half-symbol folded spectrogram (`0..=15`, `8,700 B` on handheld).
    fold_half: [u8; HALF_BYTES],
}

/// Per-minute decode statistics for on-radio Step 1 timing and LDPC iteration budgets.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FinishStats {
    pub primary_attempts: u8,
    pub primary_iters: u16,
    pub primary_decoded: u8,
    pub compact_attempts: u8,
    pub compact_iters: u16,
    pub compact_decoded: u8,
    pub total_decoded: u8,
}

/// Replay-only probe describing a known transmitted frame for miss attribution.
#[derive(Clone, Copy, Debug)]
pub struct OracleProbe {
    pub expected_frame: Frame,
    /// Transmitted base tone frequency (tone 0) in Hz.
    pub audio_hz: u16,
    /// Burst start offset from subslot `:00` in milliseconds (nominal 400 ms).
    pub offset_ms: u16,
}

/// High-level classification of why a known transmitted frame was or was not returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LossStage {
    /// Expected frame was returned in `out`.
    Returned,
    /// Blind receiver hit `MAX_DECODED` before emitting an otherwise decodable frame.
    DroppedOutputCap,
    /// Primary or compact oracle decoded the expected frame from the stored fold,
    /// so the miss occurred in acquisition, candidate selection, or guard logic.
    MissedAcquisition,
    /// Neither primary nor compact oracle decoded the expected frame from the stored
    /// fold / compact evidence.
    FailedEvidence,
}

/// Detailed per-frame trace produced by [`MinuteReceiver::finish_minute_with_oracle`].
#[derive(Clone, Copy, Debug)]
pub struct OracleAttribution {
    pub start_sym: u8,
    pub f0_half: u16,
    pub f0_hz_err: i16,
    pub nearest_channel: u8,
    pub channel_hz_err: i16,
    pub compact_representable: bool,
    pub primary_scored: bool,
    pub primary_score: i32,
    pub primary_shortlist_rank: Option<u8>,
    pub primary_suppressed: bool,
    pub primary_attempted: bool,
    pub primary_decoded_expected: bool,
    pub primary_returned_expected: bool,
    pub primary_hit_output_cap: bool,
    pub primary_duplicate_payload: bool,
    pub primary_oracle_decoded_expected: bool,
    pub compact_sync_rank: Option<u8>,
    pub compact_skipped_primary_guard: bool,
    pub compact_attempted: bool,
    pub compact_decoded_expected: bool,
    pub compact_returned_expected: bool,
    pub compact_hit_output_cap: bool,
    pub compact_oracle_decoded_expected: bool,
    pub returned_expected: bool,
}

impl OracleAttribution {
    pub const EMPTY: Self = Self {
        start_sym: 0,
        f0_half: 0,
        f0_hz_err: 0,
        nearest_channel: 0,
        channel_hz_err: 0,
        compact_representable: false,
        primary_scored: false,
        primary_score: i32::MIN,
        primary_shortlist_rank: None,
        primary_suppressed: false,
        primary_attempted: false,
        primary_decoded_expected: false,
        primary_returned_expected: false,
        primary_hit_output_cap: false,
        primary_duplicate_payload: false,
        primary_oracle_decoded_expected: false,
        compact_sync_rank: None,
        compact_skipped_primary_guard: false,
        compact_attempted: false,
        compact_decoded_expected: false,
        compact_returned_expected: false,
        compact_hit_output_cap: false,
        compact_oracle_decoded_expected: false,
        returned_expected: false,
    };

    pub fn loss_stage(&self) -> LossStage {
        if self.returned_expected {
            LossStage::Returned
        } else if (self.primary_hit_output_cap || self.compact_hit_output_cap)
            && (self.primary_oracle_decoded_expected || self.compact_oracle_decoded_expected)
        {
            LossStage::DroppedOutputCap
        } else if self.primary_oracle_decoded_expected || self.compact_oracle_decoded_expected {
            LossStage::MissedAcquisition
        } else {
            LossStage::FailedEvidence
        }
    }
}

/// Original full-band receiver for desktop applications and integrations that do not need
/// the extra nominal-channel pass.
pub type Receiver60 = MinuteReceiver<0, 0>;
/// Dual-phase handheld receiver (Layout B: 5-bit `0 ms` fold + `1,236 B` compact table
/// + 4-bit `+40 ms` half-symbol fold, `21,864 B` total).
pub type CompactReceiver60 = MinuteReceiver<COMPACT_SECONDARY_BYTES, HALF_FOLD_BYTES>;

const _: () = assert!(core::mem::size_of::<Receiver60>() + 16 <= 13 * 1024);
const _: () = assert!(core::mem::size_of::<CompactReceiver60>() + 16 <= 22 * 1024);

impl<const COMPACT_BYTES: usize, const HALF_BYTES: usize>
    MinuteReceiver<COMPACT_BYTES, HALF_BYTES>
{
    const fn blank() -> Self {
        Self {
            decim: Decimator::new(),
            pcm_ring: [0; SYM_SAMPLES],
            pcm_pos: 0,
            decim_count: 0,
            subslot: 0,
            wf_col: [0; 12],
            wf_ready: false,
            percentile_span: false,
            phase_half: false,
            fold: [0; FOLD_BYTES],
            compact: [0; COMPACT_BYTES],
            fold_half: [0; HALF_BYTES],
        }
    }

    /// Reset for a brand-new 60-second RX minute (clears folded spectrogram and sub-slot counter).
    pub fn reset(&mut self) {
        self.decim.reset();
        self.pcm_ring.fill(0);
        self.pcm_pos = 0;
        self.decim_count = 0;
        self.subslot = 0;
        self.wf_ready = false;
        self.fold.fill(0);
        self.compact.fill(0);
        self.fold_half.fill(0);
    }

    /// Begin the next 10-second sub-burst (`:00, :10, :20, :30, :40`) inside the current 60 s minute,
    /// keeping the accumulated folded spectrogram intact.
    pub fn start_subslot(&mut self) {
        self.decim.reset();
        self.pcm_ring.fill(0);
        self.pcm_pos = 0;
        self.decim_count = 0;
        self.subslot = self.subslot.saturating_add(1);
    }

    /// Configure whether this receiver samples symbols on the `+40 ms` half-symbol phase.
    pub fn set_half_symbol_phase(&mut self, half: bool) {
        self.phase_half = half;
    }
}

impl MinuteReceiver<0, 0> {
    /// Construct the original global folded-spectrum receiver.
    pub const fn new() -> Self {
        Self::blank()
    }

    /// Alternate folded spectrum for a complementary desktop decode pass.
    pub const fn with_percentile_span() -> Self {
        Self { percentile_span: true, ..Self::blank() }
    }

    /// Zero-initialize a receiver in-place without constructing a large temporary on the stack.
    ///
    /// # Safety
    /// `dst` must be valid for writes and properly aligned for `Receiver60`.
    pub unsafe fn init_in_place(dst: *mut Self) {
        core::ptr::write_bytes(dst, 0, 1);
    }
}

impl MinuteReceiver<COMPACT_SECONDARY_BYTES, HALF_FOLD_BYTES> {
    /// Construct the compact dual-phase handheld receiver (Layout B).
    pub const fn new() -> Self {
        Self::blank()
    }

    /// Zero-initialize the compact receiver in-place without constructing a large temporary on the stack.
    ///
    /// # Safety
    /// `dst` must be valid for writes and properly aligned for `CompactReceiver60`.
    pub unsafe fn init_in_place(dst: *mut Self) {
        core::ptr::write_bytes(dst, 0, 1);
    }
}

impl Default for MinuteReceiver<0, 0> {
    fn default() -> Self {
        Self::new()
    }
}
impl Default for MinuteReceiver<COMPACT_SECONDARY_BYTES, HALF_FOLD_BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const COMPACT_BYTES: usize, const HALF_BYTES: usize>
    MinuteReceiver<COMPACT_BYTES, HALF_BYTES>
{
    /// Take the latest 24-bin (`586..2,344 Hz`) packed 4-bit waterfall column if ready.
    pub fn take_waterfall_col(&mut self) -> Option<[u8; 12]> {
        if self.wf_ready {
            self.wf_ready = false;
            Some(self.wf_col)
        } else {
            None
        }
    }

    /// Diagnostic snapshot: `(completed_symbols_in_subslot, subslots_seen)`.
    pub fn diag(&self) -> (u16, u8) {
        let syms = (self.decim_count / SYM_SAMPLES as u32).min(u16::MAX as u32) as u16;
        (syms, self.subslot)
    }

    #[inline]
    fn get_cell(&self, sym: usize, bin: usize) -> u8 {
        let cell = sym * N_BINS + bin;
        let bit_pos = cell * 5;
        let byte_i = bit_pos >> 3;
        let shift = bit_pos & 7;
        let lo = (self.fold[byte_i] as u16) >> shift;
        let hi = if shift > 3 && byte_i + 1 < FOLD_BYTES {
            (self.fold[byte_i + 1] as u16) << (8 - shift)
        } else {
            0
        };
        ((lo | hi) & 0x1F) as u8
    }

    #[inline]
    fn add_cell(&mut self, sym: usize, bin: usize, delta: u8) {
        let cur = self.get_cell(sym, bin);
        let nv = cur.saturating_add(delta).min(31) as u16;
        let cell = sym * N_BINS + bin;
        let bit_pos = cell * 5;
        let byte_i = bit_pos >> 3;
        let shift = bit_pos & 7;
        let mask0 = (0x1Fu16 << shift) & 0xFF;
        self.fold[byte_i] = (self.fold[byte_i] & !(mask0 as u8)) | (((nv << shift) & 0xFF) as u8);
        if shift > 3 && byte_i + 1 < FOLD_BYTES {
            let mask1 = 0x1Fu16 >> (8 - shift);
            self.fold[byte_i + 1] =
                (self.fold[byte_i + 1] & !(mask1 as u8)) | ((nv >> (8 - shift)) as u8);
        }
    }

    #[inline]
    fn get_cell_half(&self, sym: usize, bin: usize) -> u8 {
        if HALF_BYTES < HALF_FOLD_BYTES {
            return 0;
        }
        let cell = sym * N_BINS + bin;
        let byte = self.fold_half[cell >> 1];
        if (cell & 1) == 0 {
            byte & 0x0F
        } else {
            byte >> 4
        }
    }

    #[inline]
    fn add_cell_half(&mut self, sym: usize, bin: usize, delta: u8) {
        if HALF_BYTES < HALF_FOLD_BYTES {
            return;
        }
        let cell = sym * N_BINS + bin;
        let byte_i = cell >> 1;
        let cur = if (cell & 1) == 0 {
            self.fold_half[byte_i] & 0x0F
        } else {
            self.fold_half[byte_i] >> 4
        };
        let nv = cur.saturating_add(delta).min(15);
        if (cell & 1) == 0 {
            self.fold_half[byte_i] = (self.fold_half[byte_i] & 0xF0) | nv;
        } else {
            self.fold_half[byte_i] = (self.fold_half[byte_i] & 0x0F) | (nv << 4);
        }
    }

    #[inline]
    fn quantize(log_power: i32, floor: i32, span: i32) -> i32 {
        ((log_power - floor + 3).max(0) * 6 / span).clamp(0, 6)
    }

    #[inline]
    fn quantize4(log_power: i32, floor: i32, span: i32) -> u8 {
        ((log_power - floor + 3).max(0) * 3 / span).clamp(0, 3) as u8
    }

    #[inline]
    fn compact_address(channel: usize, sym: usize) -> (usize, usize) {
        let record = channel * COMPACT_SYMS + sym - COMPACT_FIRST_SYM;
        (record * 3 / 2, (record & 1) * 4)
    }

    #[inline]
    fn compact_word(&self, byte: usize) -> u16 {
        u16::from_le_bytes([self.compact[byte], self.compact[byte + 1]])
    }

    /// Retain only the positive improvement in local tone contrast. This compact
    /// representation is sampled at published channel centers, so it needs no
    /// second FFT or general-purpose spectrum buffer.
    fn accumulate_compact(&mut self, sym: usize, bins: &[u8; N_BINS], floor: i32, peak_span: i32) {
        if COMPACT_BYTES < COMPACT_SECONDARY_BYTES
            || sym.wrapping_sub(COMPACT_FIRST_SYM) >= COMPACT_SYMS
        {
            return;
        }

        let mut histogram = [0u8; 32];
        for &bin in bins {
            histogram[usize::from(bin >> 3)] += 1;
        }
        let mut seen = 0usize;
        let mut upper = 0i32;
        for (bucket, &count) in histogram.iter().enumerate().rev() {
            seen += usize::from(count);
            if seen > N_BINS / 20 {
                upper = (bucket * 8 + 7) as i32;
                break;
            }
        }
        let local_span = (upper - floor).clamp(18, 96);
        if local_span >= peak_span {
            return;
        }

        for channel in 0..COMPACT_CHANNELS {
            let (byte, shift) = Self::compact_address(channel, sym);
            let old = self.compact_word(byte) >> shift;
            let mut record = 0u16;
            for tone in 0..4u8 {
                let position = NOMINAL_HALF_BINS[channel] * 8 + usize::from(tone) * TONE_SPACING_Q4;
                let bin = position >> 4;
                let fraction = (position & 15) as i32;
                let old_q4 = Self::quantize(i32::from(bins[bin]), floor, peak_span)
                    * (16 - fraction)
                    + Self::quantize(i32::from(bins[bin + 1]), floor, peak_span) * fraction;
                let local_q4 = Self::quantize(i32::from(bins[bin]), floor, local_span)
                    * (16 - fraction)
                    + Self::quantize(i32::from(bins[bin + 1]), floor, local_span) * fraction;
                let added = ((local_q4 - old_q4 + 8) >> 4).max(0) as u16;
                let prior = (old >> (tone * 3)) & 7;
                record |= (prior + added).min(7) << (tone * 3);
            }
            let updated = (self.compact_word(byte) & !(0x0fff << shift)) | (record << shift);
            let [lo, hi] = updated.to_le_bytes();
            self.compact[byte] = lo;
            self.compact[byte + 1] = hi;
        }
    }

    /// Push a chunk of `24,000 Hz` signed 16-bit audio samples.
    pub fn push_samples(&mut self, samples: &[i16]) {
        let origin0 = (FOLD_ORIGIN_SYMS * SYM_SAMPLES) as u32
            + if HALF_BYTES == 0 && self.phase_half {
                (SYM_SAMPLES / 2) as u32
            } else {
                0
            };
        let origin_half = (FOLD_ORIGIN_SYMS * SYM_SAMPLES + SYM_SAMPLES / 2) as u32;
        for &s in samples {
            if let Some(y) = self.decim.push(s, &LP_24K_TO_6K, 4) {
                self.pcm_ring[self.pcm_pos as usize] = y;
                self.pcm_pos = if self.pcm_pos as usize + 1 == SYM_SAMPLES {
                    0
                } else {
                    self.pcm_pos + 1
                };
                self.decim_count = self.decim_count.wrapping_add(1);
                if self.decim_count >= origin0 + SYM_SAMPLES as u32
                    && (self.decim_count - origin0).is_multiple_of(SYM_SAMPLES as u32)
                {
                    let sym = ((self.decim_count - origin0) / SYM_SAMPLES as u32) as usize - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol(sym, false);
                    }
                } else if HALF_BYTES >= HALF_FOLD_BYTES
                    && self.decim_count >= origin_half + SYM_SAMPLES as u32
                    && (self.decim_count - origin_half).is_multiple_of(SYM_SAMPLES as u32)
                {
                    let sym = ((self.decim_count - origin_half) / SYM_SAMPLES as u32) as usize - 1;
                    if sym < FOLD_SYMS {
                        self.on_symbol(sym, true);
                    }
                }
            }
        }
    }

    #[inline(never)]
    fn on_symbol(&mut self, sym: usize, half: bool) {
        // Rectangular 70 ms steady-hold window (skipping the 10 ms raised-cosine ramp) zero-padded to 512 pts.
        let mut z = [0i16; FFT_N];
        let mut idx = (self.pcm_pos as usize + RAMP_SAMPLES_6K) % SYM_SAMPLES;
        let steady_n = SYM_SAMPLES - RAMP_SAMPLES_6K; // 420 samples = 70 ms
        for sample in z.iter_mut().take(steady_n) {
            *sample = self.pcm_ring[idx];
            idx = if idx + 1 == SYM_SAMPLES { 0 } else { idx + 1 };
        }
        let shift = cfft(&mut z, FFT_M);

        let mut cur_bins = [0u8; N_BINS];
        let mut out_i = 0usize;
        let mut sum = 0u32;
        real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
            let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
            let v = lp.clamp(0, 255) as u8;
            cur_bins[out_i] = v;
            sum += v as u32;
            out_i += 1;
        });
        let floor = (sum / N_BINS as u32) as i32;
        let upper = if self.percentile_span {
            // Eight log codes per bucket. Ignore the top 5% of passband bins.
            let mut counts = [0u8; 32];
            for &v in &cur_bins {
                counts[usize::from(v >> 3)] += 1;
            }
            let mut above = 0;
            let mut upper = 0;
            for (bucket, &count) in counts.iter().enumerate().rev() {
                above += usize::from(count);
                if above > N_BINS / 20 {
                    upper = (bucket * 8 + 7) as i32;
                    break;
                }
            }
            upper
        } else {
            i32::from(cur_bins.iter().copied().max().unwrap_or(0))
        };
        let span = (upper - floor).clamp(18, 96);
        if half {
            for (b, &lp) in cur_bins.iter().enumerate() {
                let q = Self::quantize4(i32::from(lp), floor, span);
                self.add_cell_half(sym, b, q);
            }
            return;
        }
        for (b, &lp) in cur_bins.iter().enumerate() {
            let q = Self::quantize(i32::from(lp), floor, span) as u8;
            self.add_cell(sym, b, q);
        }
        self.accumulate_compact(sym, &cur_bins, floor, span);

        // Emit a 24-bin waterfall column every 3 symbols (~240 ms)
        if sym.is_multiple_of(3) {
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
    }

    #[inline]
    fn sample_tone_q4_phase(&self, sym: usize, f0_half: usize, tone: u8, half: bool) -> i32 {
        // 50 Hz tone spacing at 11.71875 Hz/bin -> 4.267 bins -> 68.27 in Q4
        let pos_x16 = f0_half * HALF_BIN_Q4 + usize::from(tone) * TONE_SPACING_Q4;
        let b = pos_x16 >> 4;
        let frac = (pos_x16 & 15) as i32;
        if b + 1 >= N_BINS || sym >= FOLD_SYMS {
            return 0;
        }
        let (v0, v1) = if half {
            (
                (self.get_cell_half(sym, b) as i32) * 2,
                (self.get_cell_half(sym, b + 1) as i32) * 2,
            )
        } else {
            (self.get_cell(sym, b) as i32, self.get_cell(sym, b + 1) as i32)
        };
        v0 * (16 - frac) + v1 * frac
    }

    #[inline]
    fn sample_tone_q4(&self, sym: usize, f0_half: usize, tone: u8) -> i32 {
        self.sample_tone_q4_phase(sym, f0_half, tone, false)
    }

    fn decode_compact_iters(
        &self,
        channel: usize,
        start_sym: u8,
        scratch: &mut BpScratch,
    ) -> (Option<Frame>, u8) {
        if COMPACT_BYTES < COMPACT_SECONDARY_BYTES {
            return (None, 0);
        }
        let f0_half = NOMINAL_HALF_BINS[channel];
        let mut interleaved = [0i8; N_BITS];
        for dibit in 0..N_DATA_SYMS {
            let symbol = usize::from(start_sym) + data_sym_pos(dibit);
            let (byte, shift) = Self::compact_address(channel, symbol);
            let corrections = self.compact_word(byte) >> shift;
            let tones: [i32; 4] = core::array::from_fn(|tone| {
                self.sample_tone_q4(symbol, f0_half, tone as u8)
                    + i32::from((corrections >> (tone * 3)) & 7) * 16
            });
            interleaved[2 * dibit] =
                (tones[0].max(tones[1]) - tones[2].max(tones[3])).clamp(-127, 127) as i8;
            interleaved[2 * dibit + 1] =
                (tones[0].max(tones[3]) - tones[1].max(tones[2])).clamp(-127, 127) as i8;
        }
        let mut deinterleaved = [0i8; N_BITS];
        let mut scale = 1i8;
        for bit in 0..N_BITS {
            let value = interleaved[interleave_idx(bit)];
            deinterleaved[bit] = value;
            scale = scale.max(value.abs());
        }
        for value in &mut deinterleaved {
            *value = (i16::from(*value) * 60 / i16::from(scale)) as i8;
        }
        let (bits, iters) = ldpc::decode_bp_iters(&deinterleaved, scratch, 35);
        (bits.and_then(|b| Frame::from_bits78(&b)), iters)
    }

    fn decode_compact(
        &self,
        channel: usize,
        start_sym: u8,
        scratch: &mut BpScratch,
    ) -> Option<Frame> {
        self.decode_compact_iters(channel, start_sym, scratch).0
    }

    /// Extract the normalized `i8` deinterleaved LLRs used by the primary pass at
    /// `(start_sym, f0_half)`. Exposed for host-side decoder-ladder comparisons.
    pub fn primary_llrs(&self, start_sym: u8, f0_half: u16) -> [i8; N_BITS] {
        self.primary_llrs_phase(start_sym, f0_half, false)
    }

    fn primary_llrs_phase(&self, start_sym: u8, f0_half: u16, half: bool) -> [i8; N_BITS] {
        let mut inter_llr = [0i16; N_BITS];
        let mut max_abs = 1i32;
        for d in 0..N_DATA_SYMS {
            let s = start_sym as usize + data_sym_pos(d);
            let m0 = self.sample_tone_q4_phase(s, f0_half as usize, 0, half);
            let m1 = self.sample_tone_q4_phase(s, f0_half as usize, 1, half);
            let m2 = self.sample_tone_q4_phase(s, f0_half as usize, 2, half);
            let m3 = self.sample_tone_q4_phase(s, f0_half as usize, 3, half);
            let b0 = m0.max(m1) - m2.max(m3);
            let b1 = m0.max(m3) - m1.max(m2);
            inter_llr[2 * d] = b0 as i16;
            inter_llr[2 * d + 1] = b1 as i16;
            max_abs = max_abs.max(b0.abs()).max(b1.abs());
        }
        let mut llr_i8 = [0i8; N_BITS];
        for i in 0..N_BITS {
            let v = inter_llr[interleave_idx(i)] as i32;
            llr_i8[i] = ((v * 60) / max_abs).clamp(-120, 120) as i8;
        }
        llr_i8
    }

    fn decode_phase_iters(
        &self,
        start_sym: u8,
        f0_half: u16,
        half: bool,
        scratch: &mut BpScratch,
    ) -> (Option<Frame>, u8) {
        let llr_i8 = self.primary_llrs_phase(start_sym, f0_half, half);
        let (bits, iters) = ldpc::decode_bp_iters(&llr_i8, scratch, 35);
        (bits.and_then(|b| Frame::from_bits78(&b)), iters)
    }

    fn decode_primary_iters(
        &self,
        start_sym: u8,
        f0_half: u16,
        scratch: &mut BpScratch,
    ) -> (Option<Frame>, u8) {
        self.decode_phase_iters(start_sym, f0_half, false, scratch)
    }

    fn decode_primary(
        &self,
        start_sym: u8,
        f0_half: u16,
        scratch: &mut BpScratch,
    ) -> Option<Frame> {
        self.decode_primary_iters(start_sym, f0_half, scratch).0
    }

    /// Run blind `S0 + S1 + S2` stacked-sync acquisition and `(184, 78)` LDPC + CRC-14 decoding
    /// across the folded 5-sub-burst spectrogram, returning up to `MAX_DECODED` unique frames.
    #[inline(never)]
    pub fn finish_minute(&mut self, out: &mut [Decoded; MAX_DECODED]) -> usize {
        self.finish_minute_with_counts(out).1
    }

    /// Returns the global-pass count followed by the combined count. For [`Receiver60`],
    /// both values are equal. [`CompactReceiver60`] adds only CRC-valid secondary frames.
    pub fn finish_minute_with_counts(
        &mut self,
        out: &mut [Decoded; MAX_DECODED],
    ) -> (usize, usize) {
        let stats = self.finish_minute_impl(out, &[], &mut []);
        (stats.primary_decoded as usize, stats.total_decoded as usize)
    }

    /// Run [`Self::finish_minute`] and return per-pass LDPC attempt and iteration counts.
    #[inline(never)]
    pub fn finish_minute_with_stats(&mut self, out: &mut [Decoded; MAX_DECODED]) -> FinishStats {
        self.finish_minute_impl(out, &[], &mut [])
    }

    /// Replay-only instrumented decode that executes the exact same blind acquisition and
    /// decoding path as [`Self::finish_minute_with_counts`], while tracing each probe frame
    /// and running a forced oracle decode at its closest representable `(start_sym, f0_half)`
    /// (and nearest nominal channel for the compact pass).
    pub fn finish_minute_with_oracle(
        &mut self,
        out: &mut [Decoded; MAX_DECODED],
        probes: &[OracleProbe],
        report: &mut [OracleAttribution],
    ) -> (usize, usize) {
        let stats = self.finish_minute_impl(out, probes, report);
        (stats.primary_decoded as usize, stats.total_decoded as usize)
    }

    fn finish_minute_impl(
        &mut self,
        out: &mut [Decoded; MAX_DECODED],
        probes: &[OracleProbe],
        report: &mut [OracleAttribution],
    ) -> FinishStats {
        assert!(report.len() >= probes.len());
        const MAX_CAND: usize = 10;
        let mut best: [(i32, u8, u16); MAX_CAND] = [(i32::MIN, 0, 0); MAX_CAND];
        let mut n_best = 0usize;
        // The fourth tone requires a pair of FFT bins for interpolation.
        let max_f0_half = ((N_BINS - 1) * 16 - 1 - 3 * TONE_SPACING_Q4) / HALF_BIN_Q4;
        let mut compact_sync = [[(i32::MIN, 0u8); 2]; COMPACT_CHANNELS];

        for (i, probe) in probes.iter().enumerate() {
            let mut attr = OracleAttribution::EMPTY;
            let raw_sym = ((probe.offset_ms as i32 - (FOLD_ORIGIN_SYMS as i32 * 80) + 40) / 80)
                .clamp(0, (FOLD_SYMS - 110) as i32) as usize;
            attr.start_sym = raw_sym as u8;
            let raw_half =
                ((probe.audio_hz as usize) * (FFT_N * 2) + 3_000) / 6_000;
            let clamped_half = raw_half.saturating_sub(FIRST_BIN * 2).min(max_f0_half);
            attr.f0_half = clamped_half as u16;
            let grid_hz = (((FIRST_BIN * 2 + clamped_half) * 6_000) + 512) / 1024;
            attr.f0_hz_err = (probe.audio_hz as i32 - grid_hz as i32) as i16;

            let mut best_ch = 0usize;
            let mut best_err = i32::MAX;
            for (ch, &ch_hz) in SUBCHANNELS_HZ.iter().enumerate() {
                let err = probe.audio_hz as i32 - ch_hz as i32;
                if err.abs() < best_err.abs() {
                    best_err = err;
                    best_ch = ch;
                }
            }
            attr.nearest_channel = best_ch as u8;
            attr.channel_hz_err = best_err as i16;
            attr.compact_representable =
                COMPACT_BYTES == COMPACT_SECONDARY_BYTES && best_err.abs() <= 6;
            report[i] = attr;
        }

        for start_sym in 0usize..=(FOLD_SYMS - 110) {
            let mut nominal_channel = 0usize;
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

                for attr in report.iter_mut().take(probes.len()) {
                    if attr.start_sym as usize == start_sym && attr.f0_half as usize == f0_half {
                        attr.primary_scored = true;
                        attr.primary_score = score;
                    }
                }

                if COMPACT_BYTES == COMPACT_SECONDARY_BYTES
                    && start_sym >= 1
                    && nominal_channel < COMPACT_CHANNELS
                    && f0_half == NOMINAL_HALF_BINS[nominal_channel]
                {
                    let best_starts = &mut compact_sync[nominal_channel];
                    if score > best_starts[0].0 {
                        best_starts[1] = best_starts[0];
                        best_starts[0] = (score, start_sym as u8);
                    } else if score > best_starts[1].0 {
                        best_starts[1] = (score, start_sym as u8);
                    }
                    nominal_channel += 1;
                }

                if n_best == MAX_CAND && score <= best[MAX_CAND - 1].0 {
                    continue;
                }
                // Suppress near-duplicates within +-1 start_sym and +-4 half-bins
                // (prevents one strong caller from consuming multiple shortlist slots).
                let mut dup_idx = None;
                for (i, item) in best.iter().enumerate().take(n_best) {
                    if item.1.abs_diff(start_sym as u8) <= 1
                        && (item.2 as usize).abs_diff(f0_half) <= 4
                    {
                        dup_idx = Some(i);
                        break;
                    }
                }
                if let Some(di) = dup_idx {
                    if score > best[di].0 {
                        best[di] = (score, start_sym as u8, f0_half as u16);
                    } else {
                        for attr in report.iter_mut().take(probes.len()) {
                            if attr.start_sym as usize == start_sym
                                && attr.f0_half as usize == f0_half
                            {
                                attr.primary_suppressed = true;
                            }
                        }
                    }
                } else if n_best < MAX_CAND {
                    best[n_best] = (score, start_sym as u8, f0_half as u16);
                    n_best += 1;
                } else {
                    best[MAX_CAND - 1] = (score, start_sym as u8, f0_half as u16);
                }
                // Insertion sort descending by score
                for i in 1..n_best {
                    let mut j = i;
                    while j > 0 && best[j].0 > best[j - 1].0 {
                        best.swap(j, j - 1);
                        j -= 1;
                    }
                }
            }
        }

        // Record shortlist ranks for probes
        for attr in report.iter_mut().take(probes.len()) {
            for (rank, &(_, cand_sym, cand_half)) in best.iter().take(n_best).enumerate() {
                if cand_sym == attr.start_sym && cand_half.abs_diff(attr.f0_half) <= 3 {
                    attr.primary_shortlist_rank = Some(rank as u8);
                    break;
                }
            }
            if COMPACT_BYTES == COMPACT_SECONDARY_BYTES && attr.start_sym >= 1 {
                let starts = compact_sync[attr.nearest_channel as usize];
                for rank in 0..2 {
                    if starts[rank].1 == attr.start_sym {
                        attr.compact_sync_rank = Some(rank as u8);
                        break;
                    }
                }
            }
        }

        let mut scratch = BpScratch::new();
        let mut stats = FinishStats::default();
        let mut n_out = 0usize;
        for (cand_idx, &(score, start_sym, f0_half)) in best.iter().take(n_best).enumerate() {
            if n_out >= MAX_DECODED {
                for &(_, rem_sym, rem_half) in best[cand_idx..n_best].iter() {
                    for attr in report.iter_mut().take(probes.len()) {
                        if rem_sym == attr.start_sym && rem_half.abs_diff(attr.f0_half) <= 3 {
                            attr.primary_hit_output_cap = true;
                        }
                    }
                }
                break;
            }
            for attr in report.iter_mut().take(probes.len()) {
                if start_sym == attr.start_sym && f0_half.abs_diff(attr.f0_half) <= 3 {
                    attr.primary_attempted = true;
                }
            }
            stats.primary_attempts = stats.primary_attempts.saturating_add(1);
            let (decoded_frame, iters) = self.decode_primary_iters(start_sym, f0_half, &mut scratch);
            stats.primary_iters = stats.primary_iters.saturating_add(u16::from(iters));
            if let Some(frame) = decoded_frame {
                let packed = frame.pack_u64();
                let is_dup = out[..n_out].iter().any(|d| d.frame.pack_u64() == packed);
                for (p_idx, probe) in probes.iter().enumerate() {
                    if frame == probe.expected_frame {
                        report[p_idx].primary_decoded_expected = true;
                        if is_dup {
                            report[p_idx].primary_duplicate_payload = true;
                        } else {
                            report[p_idx].primary_returned_expected = true;
                        }
                    }
                }
                if is_dup {
                    continue;
                }
                // Convert half-bin index back to Hz above USB dial: (FIRST_BIN + f0_half/2) * 6000 / 512
                let audio_hz = (((FIRST_BIN as u32 * 2 + f0_half as u32) * 6000) / 1024) as u16;
                let time_offset_ms = (start_sym as i16 - NOMINAL_FOLD_START_SYM) * 80
                    + if self.phase_half { 40 } else { 0 };
                // Rough SNR estimate in dB (2500 Hz ref) from stacked sync score
                let snr_db = (-22 + (score / 180)).clamp(-24, 10) as i8;
                out[n_out] = Decoded {
                    frame,
                    audio_hz,
                    time_offset_ms,
                    snr_db,
                };
                n_out += 1;
            }
        }
        stats.primary_decoded = n_out as u8;
        if COMPACT_BYTES == COMPACT_SECONDARY_BYTES {
            for rank in 0..2 {
                for (channel, starts) in compact_sync.iter().enumerate() {
                    let (score, start_sym) = starts[rank];
                    if n_out == MAX_DECODED {
                        for attr in report.iter_mut().take(probes.len()) {
                            if attr.nearest_channel as usize == channel
                                && attr.start_sym == start_sym
                            {
                                attr.compact_hit_output_cap = true;
                            }
                        }
                        break;
                    }
                    let time_offset_ms = (start_sym as i16 - NOMINAL_FOLD_START_SYM) * 80
                        + if self.phase_half { 40 } else { 0 };
                    if out[..n_out].iter().any(|decoded| {
                        decoded.audio_hz.abs_diff(SUBCHANNELS_HZ[channel]) <= 18
                            && decoded.time_offset_ms == time_offset_ms
                    }) {
                        for attr in report.iter_mut().take(probes.len()) {
                            if attr.nearest_channel as usize == channel
                                && attr.start_sym == start_sym
                            {
                                attr.compact_skipped_primary_guard = true;
                            }
                        }
                        continue;
                    }
                    for attr in report.iter_mut().take(probes.len()) {
                        if attr.nearest_channel as usize == channel && attr.start_sym == start_sym {
                            attr.compact_attempted = true;
                        }
                    }
                    stats.compact_attempts = stats.compact_attempts.saturating_add(1);
                    let (decoded_frame, iters) =
                        self.decode_compact_iters(channel, start_sym, &mut scratch);
                    stats.compact_iters = stats.compact_iters.saturating_add(u16::from(iters));
                    if let Some(frame) = decoded_frame {
                        let is_dup = out[..n_out].iter().any(|decoded| decoded.frame == frame);
                        for (p_idx, probe) in probes.iter().enumerate() {
                            if frame == probe.expected_frame {
                                report[p_idx].compact_decoded_expected = true;
                                if !is_dup {
                                    report[p_idx].compact_returned_expected = true;
                                }
                            }
                        }
                        if !is_dup {
                            out[n_out] = Decoded {
                                frame,
                                audio_hz: SUBCHANNELS_HZ[channel],
                                time_offset_ms,
                                snr_db: (-22 + score / 180).clamp(-24, 10) as i8,
                            };
                            n_out += 1;
                            stats.compact_decoded = stats.compact_decoded.saturating_add(1);
                        }
                    }
                }
            }
        }
        // Layout B second timing phase (+40 ms half-symbol 4-bit fold): eliminates the
        // 30..50 ms (mod 80 ms) half-symbol dead zone without touching Phase 0 evidence.
        if HALF_BYTES >= HALF_FOLD_BYTES && n_out < MAX_DECODED {
            best = [(i32::MIN, 0, 0); MAX_CAND];
            n_best = 0;
            for start_sym in 0usize..=(FOLD_SYMS - 110) {
                for f0_half in 0..=max_f0_half {
                    let mut score = 0i32;
                    for (k, &t) in S0.iter().enumerate() {
                        let s = start_sym + k;
                        let sig = self.sample_tone_q4_phase(s, f0_half, t, true);
                        let bg = (self.sample_tone_q4_phase(s, f0_half, 0, true)
                            + self.sample_tone_q4_phase(s, f0_half, 1, true)
                            + self.sample_tone_q4_phase(s, f0_half, 2, true)
                            + self.sample_tone_q4_phase(s, f0_half, 3, true))
                            >> 2;
                        score += sig - bg;
                    }
                    for (k, &t) in S1.iter().enumerate() {
                        let s = start_sym + 18 + k;
                        let sig = self.sample_tone_q4_phase(s, f0_half, t, true);
                        let bg = (self.sample_tone_q4_phase(s, f0_half, 0, true)
                            + self.sample_tone_q4_phase(s, f0_half, 1, true)
                            + self.sample_tone_q4_phase(s, f0_half, 2, true)
                            + self.sample_tone_q4_phase(s, f0_half, 3, true))
                            >> 2;
                        score += sig - bg;
                    }
                    for (k, &t) in S2.iter().enumerate() {
                        let s = start_sym + 104 + k;
                        let sig = self.sample_tone_q4_phase(s, f0_half, t, true);
                        let bg = (self.sample_tone_q4_phase(s, f0_half, 0, true)
                            + self.sample_tone_q4_phase(s, f0_half, 1, true)
                            + self.sample_tone_q4_phase(s, f0_half, 2, true)
                            + self.sample_tone_q4_phase(s, f0_half, 3, true))
                            >> 2;
                        score += sig - bg;
                    }
                    if n_best == MAX_CAND && score <= best[MAX_CAND - 1].0 {
                        continue;
                    }
                    let mut dup_idx = None;
                    for (i, item) in best.iter().enumerate().take(n_best) {
                        if item.1.abs_diff(start_sym as u8) <= 1
                            && (item.2 as usize).abs_diff(f0_half) <= 4
                        {
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
            for &(score, start_sym, f0_half) in best.iter().take(n_best) {
                if n_out >= MAX_DECODED {
                    break;
                }
                stats.primary_attempts = stats.primary_attempts.saturating_add(1);
                let (decoded_frame, iters) =
                    self.decode_phase_iters(start_sym, f0_half, true, &mut scratch);
                stats.primary_iters = stats.primary_iters.saturating_add(u16::from(iters));
                if let Some(frame) = decoded_frame {
                    let packed = frame.pack_u64();
                    if out[..n_out].iter().any(|d| d.frame.pack_u64() == packed) {
                        continue;
                    }
                    let audio_hz = (((FIRST_BIN as u32 * 2 + f0_half as u32) * 6000) / 1024) as u16;
                    let time_offset_ms = (start_sym as i16 - NOMINAL_FOLD_START_SYM) * 80 + 40;
                    let snr_db = (-22 + (score / 180)).clamp(-24, 10) as i8;
                    out[n_out] = Decoded {
                        frame,
                        audio_hz,
                        time_offset_ms,
                        snr_db,
                    };
                    n_out += 1;
                }
            }
        }
        stats.total_decoded = n_out as u8;

        // Forced oracle decodes for each probe (bypassing candidate ranking, guards, and output cap).
        for (p_idx, probe) in probes.iter().enumerate() {
            let attr = &mut report[p_idx];
            attr.returned_expected = out[..n_out]
                .iter()
                .any(|d| d.frame == probe.expected_frame);
            if let Some(frame) = self.decode_primary(attr.start_sym, attr.f0_half, &mut scratch) {
                attr.primary_oracle_decoded_expected = frame == probe.expected_frame;
            }
            if COMPACT_BYTES == COMPACT_SECONDARY_BYTES && attr.start_sym >= 1 {
                if let Some(frame) =
                    self.decode_compact(attr.nearest_channel as usize, attr.start_sym, &mut scratch)
                {
                    attr.compact_oracle_decoded_expected = frame == probe.expected_frame;
                }
            }
        }
        stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::encode_grid4;
    use crate::phy::{encode_tones, Modulator, DEFAULT_SPACING_HZ};
    use crate::rx::INPUT_RATE_HZ;
    use crate::Callsign;

    struct Rng(u64);
    impl Rng {
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            (x >> 16) as u32
        }
        fn gauss(&mut self, sigma: f64) -> f64 {
            let mut s = 0.0;
            for _ in 0..12 {
                s += (self.next_u32() as f64) / (u32::MAX as f64 + 1.0);
            }
            (s - 6.0) * sigma
        }
    }

    #[test]
    fn receiver60_size_and_blind_weak_signal_decode() {
        assert!(core::mem::size_of::<Receiver60>() + 16 <= 13 * 1024);

        let nq6n = Callsign::encode(b"NQ6N").unwrap();
        let en61 = encode_grid4(b"EN61").unwrap();
        let frame = Frame::Cq {
            sender: nq6n,
            grid4: en61,
            tx_pwr: 11,
            reserved: 0,
        };
        let tones = encode_tones(frame);

        let sigma = 2000.0f64;
        let p_n_2500 = sigma * sigma * (2500.0 / (INPUT_RATE_HZ as f64 / 2.0));
        let mut rng = Rng(0x9506_0001_CAFE_BABE);
        let mut rx = Receiver60::new();

        // Verify blind decode at -19 dB (2500 Hz ref) across 5x 10s sub-bursts
        let amp = (2.0 * p_n_2500 * 10.0f64.powf(-19.0 / 10.0)).sqrt() as i16;
        for hz in [1050, 2175] {
            let mut ok = 0usize;
            for _ in 0..6 {
                rx.reset();
                for sub in 0..5 {
                    if sub > 0 {
                        rx.start_subslot();
                    }
                    let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
                    for s in &mut pcm {
                        *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
                    }
                    let mut m = Modulator::new(tones, INPUT_RATE_HZ, hz, DEFAULT_SPACING_HZ, amp);
                    let mut idx = (400 * INPUT_RATE_HZ as usize) / 1000;
                    while let Some(v) = m.next_sample() {
                        pcm[idx] = pcm[idx].saturating_add(v);
                        idx += 1;
                    }
                    for chunk in pcm.chunks(192) {
                        rx.push_samples(chunk);
                    }
                }
                let mut out = [Decoded::EMPTY; MAX_DECODED];
                let n = rx.finish_minute(&mut out);
                if out[..n].iter().any(|d| d.frame == frame) {
                    ok += 1;
                }
            }
            assert!(ok >= 5, "Receiver60 blind decode at {hz} Hz and -19 dB too low: {ok}/6");
        }

        // Noise-only false-accept control (10 full 50s trials)
        let mut fa = 0usize;
        for _ in 0..10 {
            rx.reset();
            for sub in 0..5 {
                if sub > 0 {
                    rx.start_subslot();
                }
                let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
                for s in &mut pcm {
                    *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
                }
                for chunk in pcm.chunks(192) {
                    rx.push_samples(chunk);
                }
            }
            let mut out = [Decoded::EMPTY; MAX_DECODED];
            if rx.finish_minute(&mut out) > 0 {
                fa += 1;
            }
        }
        assert_eq!(fa, 0, "Receiver60 noise-only false accepts must be 0");
    }
}
