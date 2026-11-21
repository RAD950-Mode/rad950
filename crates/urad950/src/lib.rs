//! # urad950
//!
//! Reference `no_std`, integer-only implementation of the **RAD950** 10m weak-signal 4-FSK digital mode:
//! - [`frame`]: 64-bit `CQ` and `RX` (`R+RX`) frame codec, 28-bit callsign packing, `GRID4`, `SNR`, and CRC-14.
//! - [`ldpc`]: Girth-6 `(184, 78)` systematic PEG LDPC encoder and normalized min-sum belief-propagation decoder.
//! - [`phy`]: `6 + 6 + 6` 4-ary sync sequences, stride-37 bit interleaver, Gray tone map, and reproducible
//!   phase-continuous TX modulator (`10 ms` raised-cosine ramp + `70 ms` steady hold).
//! - [`rx`]: Streaming receiver (`<= 13,312` bytes, zero heap) with 4:1 FIR decimation (`24 kHz -> 6 kHz`),
//!   480-sample Hann-windowed 512-pt FFT, `±320 ms` (`±4` symbol) early-acquisition ring buffer, and `K = 8` tracks.

#![no_std]
#![allow(clippy::needless_range_loop)]

pub mod chat;
pub mod distress;
pub mod ext;
pub mod frame;
pub mod ldpc;
pub mod phy;
pub mod rx;
pub mod rx60;

pub use ext::{encode_ext_cq, encode_ext_rx, ExtMsg, ExtReassembler, StationId};
pub use frame::{CallError, Callsign, Frame, GRID_MAX, GRID_UNKNOWN};
pub use phy::{encode_tones, Modulator, DEFAULT_SPACING_HZ, N_SYMBOLS, SUBCHANNELS_HZ};
pub use rx::{Decoded, Receiver, SoftCandidate, MAX_DECODED};
pub use rx60::Receiver60;

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{crc14_u64, decode_grid4, decode_snr_db, encode_grid4, encode_snr_db};
    use crate::phy::{reg70_gain_at, S0, S1, S2, SUBSTEPS_RAMP, SUBSTEPS_SYM};
    use crate::rx::{INPUT_RATE_HZ, NOMINAL_START_SYM};

    /// Ferrite's shared `Decoder` enum slot budget (`crates/ferrite-core/src/decoders.rs::BUDGET`).
    const DECODER_BUDGET: usize = 13 * 1024;

    /// Deterministic PRNG (xorshift64) + Box-Muller-like 12-uniform Gaussian sum for reproducible AWGN tests.
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

        /// Zero-mean unit-variance approximate Gaussian scaled by `sigma`.
        fn gauss(&mut self, sigma: f64) -> f64 {
            // Sum of 12 uniforms on [0, 1) minus 6 has mean 0 and variance 1.
            let mut s = 0.0;
            for _ in 0..12 {
                s += (self.next_u32() as f64) / (u32::MAX as f64 + 1.0);
            }
            (s - 6.0) * sigma
        }
    }

    #[test]
    fn memory_budget_fits_decoder_slot() {
        let sz = core::mem::size_of::<Receiver>();
        assert!(sz <= DECODER_BUDGET, "Receiver is {sz} bytes, exceeds {DECODER_BUDGET} byte budget");
    }

    #[test]
    fn callsign_grid_and_snr_codec() {
        for &call in &[b"NQ6N".as_slice(), b"K1ABC", b"VE3XYZ", b"G4ABC", b"3D2AG", b"9A1A"] {
            let c = Callsign::encode(call).unwrap();
            let mut buf = [0u8; 6];
            let n = c.decode(&mut buf);
            assert_eq!(&buf[..n], call);
        }
        assert_eq!(Callsign::encode(b""), Err(CallError::Empty));
        assert_eq!(Callsign::encode(b"W1AW/P"), Err(CallError::InvalidChar));
        assert_eq!(Callsign::encode(b"WA1ABCD"), Err(CallError::TooLong));
        assert_eq!(Callsign::encode(b"NOCALL"), Err(CallError::NoAreaDigit));

        assert_eq!(encode_grid4(b"AA00"), Some(0));
        assert_eq!(encode_grid4(b"RR99"), Some(GRID_MAX));
        let fn31 = encode_grid4(b"FN31").unwrap();
        assert_eq!(decode_grid4(fn31), Some(*b"FN31"));
        assert_eq!(decode_grid4(GRID_UNKNOWN), None);

        assert_eq!(decode_snr_db(encode_snr_db(-30)), -24);
        assert_eq!(decode_snr_db(encode_snr_db(-16)), -16);
        assert_eq!(decode_snr_db(encode_snr_db(0)), 0);
        assert_eq!(decode_snr_db(encode_snr_db(15)), 7);
    }

    #[test]
    fn sync_sequences_satisfy_ambiguity_bounds() {
        for w in [&S0, &S1, &S2] {
            for t in 0..4u8 {
                assert!(w.contains(&t), "all 4 tones must appear");
            }
            for i in 0..5 {
                assert_ne!(w[i], w[i + 1], "no adjacent repeated tones");
            }
            for dt in -5i32..=5 {
                for df in -3i32..=3 {
                    if dt == 0 && df == 0 {
                        continue;
                    }
                    let hits = (0..6i32)
                        .filter(|&i| (0..6).contains(&(i + dt)) && w[i as usize] as i32 + df == w[(i + dt) as usize] as i32)
                        .count();
                    assert!(hits <= 1, "auto-ambiguity sidelobe {hits} > 1 at dt={dt}, df={df}");
                }
            }
        }
        // Joint S0 (0..6) + S1 (18..24) 12-symbol 2D ambiguity <= 2 everywhere off (0, 0), and 0 at dt=0, df!=0.
        let mut pts = [(0i32, 0i32); 12];
        for i in 0..6 {
            pts[i] = (i as i32, S0[i] as i32);
            pts[6 + i] = (18 + i as i32, S1[i] as i32);
        }
        for dt in -24i32..=24 {
            for df in -3i32..=3 {
                if dt == 0 && df == 0 {
                    continue;
                }
                let hits = pts.iter().filter(|&&(p, f)| pts.contains(&(p + dt, f + df))).count();
                if dt == 0 {
                    assert_eq!(hits, 0, "aligned frequency shift must have 0 sync hits");
                }
                assert!(hits <= 2, "joint S0+S1 sidelobe {hits} > 2 at dt={dt}, df={df}");
            }
        }
    }

    #[test]
    fn worked_reference_vectors_and_tx_trajectory() {
        let nq6n = Callsign::encode(b"NQ6N").unwrap();
        let k1abc = Callsign::encode(b"K1ABC").unwrap();
        let fn31 = encode_grid4(b"FN31").unwrap();

        let cq = Frame::Cq { sender: nq6n, grid4: fn31, tx_pwr: 10, reserved: 0 };
        let rx_init = Frame::Rx { sender: k1abc, recipient: nq6n, snr_db: -14, ack: false, reserved: false };
        let rx_ack = Frame::Rx { sender: nq6n, recipient: k1abc, snr_db: -11, ack: true, reserved: false };

        // Lock deterministic reference payload words, CRCs, and all 110 tone indices (matching rad950-design-v2.md §3).
        let cq_u64 = cq.pack_u64();
        let rx_u64 = rx_init.pack_u64();
        assert_eq!(cq_u64, 0x30DD_C102_85BA_0000);
        assert_eq!(crc14_u64(cq_u64), 0x0648);
        assert_eq!(rx_u64, 0x80F2_C50B_0DDC_1028);
        assert_eq!(crc14_u64(rx_u64), 0x2375);
        assert_eq!(Frame::unpack_u64(cq_u64), Some(cq));
        assert_ne!(crc14_u64(0), 0, "all-zero payload must not have zero CRC");

        let cq_tones = encode_tones(cq);
        let rx_tones = encode_tones(rx_init);
        let ack_tones = encode_tones(rx_ack);
        let want_cq = b"03013201033000332130320110001201300301003132130333202301032213222031210100301212221033100130032203101110310230";
        let want_rx = b"03013233032031213030320122013131212101030023013320101000213221002133313103202003203320130130210302123202310230";
        for i in 0..N_SYMBOLS {
            assert_eq!(cq_tones[i], want_cq[i] - b'0', "CQ tone {i}");
            assert_eq!(rx_tones[i], want_rx[i] - b'0', "RX tone {i}");
        }
        assert_eq!(&cq_tones[0..6], &S0);
        assert_eq!(&cq_tones[18..24], &S1);
        assert_eq!(&cq_tones[104..110], &S2);

        // Verify TX trajectory: 10 ms (8 sub-steps) smooth transition + 70 ms (56 sub-steps) steady hold.
        let gains = [16u8, 48, 80, 112];
        for sym in 1..N_SYMBOLS {
            let g_end_ramp = reg70_gain_at(&cq_tones, sym, SUBSTEPS_RAMP, gains);
            let g_target = gains[cq_tones[sym] as usize];
            assert_eq!(g_end_ramp, g_target);
            for sub in SUBSTEPS_RAMP..SUBSTEPS_SYM {
                assert_eq!(reg70_gain_at(&cq_tones, sym, sub, gains), g_target);
            }
        }

        // End-to-end clean loopback through Modulator -> 192-sample (8 ms) streaming blocks -> Receiver.
        for &(expected, f_hz, offset_ms) in &[(cq, 1050u32, 0i32), (rx_init, 1500, -160), (rx_ack, 1950, 200)] {
            let tones = match expected {
                f if f == cq => cq_tones,
                f if f == rx_init => rx_tones,
                _ => ack_tones,
            };
            let mut slot_pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
            let start_ms = (NOMINAL_START_SYM as i32 * 80 + offset_ms) as usize;
            let start_idx = start_ms * (INPUT_RATE_HZ as usize) / 1000;
            let mut m = Modulator::new(tones, INPUT_RATE_HZ, f_hz, DEFAULT_SPACING_HZ, 8_000);
            let mut idx = start_idx;
            while let Some(s) = m.next_sample() {
                slot_pcm[idx] = s;
                idx += 1;
            }

            let mut rx = Receiver::new();
            for chunk in slot_pcm.chunks(192) {
                rx.push_samples(chunk);
            }
            let mut out = [Decoded {
                frame: cq,
                audio_hz: 0,
                time_offset_ms: 0,
                snr_db: 0,
            }; MAX_DECODED];
            let n = rx.finish_slot(&mut out);
            assert_eq!(n, 1, "expected 1 decoded frame for {expected:?}");
            assert_eq!(out[0].frame, expected);
            assert!((out[0].audio_hz as i32 - f_hz as i32).abs() <= 8, "freq {} vs {f_hz}", out[0].audio_hz);
            assert!((out[0].time_offset_ms as i32 - offset_ms).abs() <= 40, "time {} vs {offset_ms}", out[0].time_offset_ms);
        }
    }

    #[test]
    fn blind_overlapping_callers_with_skew_and_strong_neighbor_displacement() {
        let w1aw = Callsign::encode(b"NQ6N").unwrap();
        let k1abc = Callsign::encode(b"K1ABC").unwrap();
        let ve3xyz = Callsign::encode(b"VE3XYZ").unwrap();

        let weak_frame = Frame::Rx { sender: k1abc, recipient: w1aw, snr_db: -15, ack: false, reserved: false };
        let loud_frame = Frame::Rx { sender: ve3xyz, recipient: w1aw, snr_db: 2, ack: false, reserved: false };

        let weak_tones = encode_tones(weak_frame);
        let loud_tones = encode_tones(loud_frame);

        // Part A: Single +15 dB strong caller at 1050 Hz in noise — verify ZERO ghost decodes at 1000 Hz or 1100 Hz
        // (+/- 1 tone spacing) and exact decode of the caller itself.
        let mut rng = Rng(0xCAFE_BABE_1234_5678);
        let mut slot_a = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
        for s in &mut slot_a {
            *s = rng.gauss(300.0) as i16;
        }
        let loud_start = ((400 + 240) * INPUT_RATE_HZ as usize) / 1000; // +240 ms late
        let mut m_loud = Modulator::new(loud_tones, INPUT_RATE_HZ, 1050, DEFAULT_SPACING_HZ, 9_000);
        let mut i = loud_start;
        while let Some(v) = m_loud.next_sample() {
            slot_a[i] = slot_a[i].saturating_add(v);
            i += 1;
        }
        let mut rx = Receiver::new();
        for chunk in slot_a.chunks(192) {
            rx.push_samples(chunk);
        }
        let mut out = [Decoded { frame: weak_frame, audio_hz: 0, time_offset_ms: 0, snr_db: 0 }; MAX_DECODED];
        let n_a = rx.finish_slot(&mut out);
        assert_eq!(n_a, 1, "strong caller must not create +/- 50 Hz ghost decodes");
        assert_eq!(out[0].frame, loud_frame);

        // Part B: Two simultaneous overlapping callers with 520 ms relative skew (-280 ms early vs +240 ms late)
        // and 18 dB power difference (weak amp = 1,100 vs loud amp = 8,800) on adjacent 225 Hz subchannels
        // (1050 Hz and 1275 Hz — separated by only 1 guard tone!).
        let mut slot_b = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
        for s in &mut slot_b {
            *s = rng.gauss(350.0) as i16;
        }
        let weak_start = ((400 - 280) * INPUT_RATE_HZ as usize) / 1000; // -280 ms early (3.5 symbols early!)
        let mut m_weak = Modulator::new(weak_tones, INPUT_RATE_HZ, 1050, DEFAULT_SPACING_HZ, 1_100);
        let mut idx = weak_start;
        while let Some(v) = m_weak.next_sample() {
            slot_b[idx] = slot_b[idx].saturating_add(v);
            idx += 1;
        }
        let mut m_loud2 = Modulator::new(loud_tones, INPUT_RATE_HZ, 1275, DEFAULT_SPACING_HZ, 8_800);
        idx = loud_start;
        while let Some(v) = m_loud2.next_sample() {
            slot_b[idx] = slot_b[idx].saturating_add(v);
            idx += 1;
        }

        rx.reset();
        for chunk in slot_b.chunks(192) {
            rx.push_samples(chunk);
        }
        let n_b = rx.finish_slot(&mut out);
        assert_eq!(n_b, 2, "both weak early caller and +18 dB late neighbor must decode simultaneously");
        assert!(out[..n_b].iter().any(|d| d.frame == weak_frame));
        assert!(out[..n_b].iter().any(|d| d.frame == loud_frame));

        // Part C1: Zero-guard 200 Hz spacing (1000 Hz vs 1200 Hz — adjacent 4-tone blocks with 0 Hz guard)
        // with 480 ms timing offset and +15 dB stronger neighbor: both acquire and decode cleanly.
        let mut slot_c1 = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
        for s in &mut slot_c1 {
            *s = rng.gauss(300.0) as i16;
        }
        let mut m_c1 = Modulator::new(weak_tones, INPUT_RATE_HZ, 1000, DEFAULT_SPACING_HZ, 1_400);
        idx = ((400 - 240) * INPUT_RATE_HZ as usize) / 1000;
        while let Some(v) = m_c1.next_sample() {
            slot_c1[idx] = slot_c1[idx].saturating_add(v);
            idx += 1;
        }
        let mut m_c2 = Modulator::new(loud_tones, INPUT_RATE_HZ, 1200, DEFAULT_SPACING_HZ, 8_000);
        idx = ((400 + 240) * INPUT_RATE_HZ as usize) / 1000;
        while let Some(v) = m_c2.next_sample() {
            slot_c1[idx] = slot_c1[idx].saturating_add(v);
            idx += 1;
        }
        rx.reset();
        for chunk in slot_c1.chunks(192) {
            rx.push_samples(chunk);
        }
        let n_c1 = rx.finish_slot(&mut out);
        assert_eq!(n_c1, 2, "at 200 Hz (4-tone) spacing with +15 dB neighbor, both callers decode");
        assert!(out[..n_c1].iter().any(|d| d.frame == weak_frame));
        assert!(out[..n_c1].iter().any(|d| d.frame == loud_frame));

        // Part C2: Reviewer's 1-tone (50 Hz) overlap test (1000 Hz vs 1050 Hz, sharing 3 of 4 tones) with +6 dB
        // stronger caller: demonstrates why 50 Hz subchannels collide on data symbols while confirming the stronger
        // caller still acquires/decodes cleanly with zero false frames.
        let mut slot_c2 = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
        for s in &mut slot_c2 {
            *s = rng.gauss(250.0) as i16;
        }
        let mut m_o1 = Modulator::new(weak_tones, INPUT_RATE_HZ, 1000, DEFAULT_SPACING_HZ, 2_500);
        idx = ((400 - 240) * INPUT_RATE_HZ as usize) / 1000;
        while let Some(v) = m_o1.next_sample() {
            slot_c2[idx] = slot_c2[idx].saturating_add(v);
            idx += 1;
        }
        let mut m_o2 = Modulator::new(loud_tones, INPUT_RATE_HZ, 1050, DEFAULT_SPACING_HZ, 5_000);
        idx = ((400 + 240) * INPUT_RATE_HZ as usize) / 1000;
        while let Some(v) = m_o2.next_sample() {
            slot_c2[idx] = slot_c2[idx].saturating_add(v);
            idx += 1;
        }
        rx.reset();
        for chunk in slot_c2.chunks(192) {
            rx.push_samples(chunk);
        }
        let n_c2 = rx.finish_slot(&mut out);
        assert!(n_c2 >= 1 && out[..n_c2].iter().any(|d| d.frame == loud_frame));
        assert!(out[..n_c2].iter().all(|d| d.frame == weak_frame || d.frame == loud_frame));
    }

    #[test]
    fn weak_signal_awgn_blind_decode_and_noise_rejection() {
        let w1aw = Callsign::encode(b"NQ6N").unwrap();
        let fn31 = encode_grid4(b"FN31").unwrap();
        let frame = Frame::Cq { sender: w1aw, grid4: fn31, tx_pwr: 10, reserved: 0 };
        let tones = encode_tones(frame);

        // At Fs = 24,000 Hz, total Nyquist noise BW is 12,000 Hz.
        // For noise_sigma = 1,200: noise power in 12 kHz is 1,200^2, so noise power in 2,500 Hz is
        // P_n(2500) = 1,200^2 * (2500 / 12000) = 300,000.
        // At SNR_2500 = -15.0 dB (0.031623), signal power P_s = A^2 / 2 = 300,000 * 0.031623 = 9,487 -> A = 138!
        let sigma = 1_200.0;
        let snr_db = -15.0f64;
        let p_n_2500 = sigma * sigma * (2500.0 / 12000.0);
        let amp = (2.0 * p_n_2500 * libm_pow10(snr_db / 10.0)).sqrt() as i16;

        let mut rng = Rng(0x1234_5678_9ABC_DEF0);
        let mut rx = Receiver::new();
        let mut decoded_ok = 0usize;
        let trials = 20usize;

        for t in 0..trials {
            let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
            for s in &mut pcm {
                *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
            }
            // Non-bin-centered frequency (1,063 Hz) and non-zero timing skew (+120 ms)
            let start_idx = ((400 + 120) * INPUT_RATE_HZ as usize) / 1000;
            let mut m = Modulator::new(tones, INPUT_RATE_HZ, 1063 + (t as u32 % 7), DEFAULT_SPACING_HZ, amp);
            let mut idx = start_idx;
            while let Some(v) = m.next_sample() {
                pcm[idx] = pcm[idx].saturating_add(v);
                idx += 1;
            }
            rx.reset();
            for chunk in pcm.chunks(192) {
                rx.push_samples(chunk);
            }
            let mut out = [Decoded { frame, audio_hz: 0, time_offset_ms: 0, snr_db: 0 }; MAX_DECODED];
            let n = rx.finish_slot(&mut out);
            if out[..n].iter().any(|d| d.frame == frame) {
                decoded_ok += 1;
            }
        }
        assert!(decoded_ok >= 16, "blind -15 dB AWGN decode rate too low: {decoded_ok}/{trials}");

        // Pure noise slots: assert 0 false frames across 25 independent slots.
        let mut false_frames = 0usize;
        for _ in 0..25 {
            let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
            for s in &mut pcm {
                *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
            }
            rx.reset();
            for chunk in pcm.chunks(192) {
                rx.push_samples(chunk);
            }
            let mut out = [Decoded { frame, audio_hz: 0, time_offset_ms: 0, snr_db: 0 }; MAX_DECODED];
            false_frames += rx.finish_slot(&mut out);
        }
        assert_eq!(false_frames, 0, "unexpected false frames on pure noise");
    }

    #[test]
    fn snr_calibration_regression_across_levels() {
        let w1aw = Callsign::encode(b"NQ6N").unwrap();
        let fn31 = encode_grid4(b"FN31").unwrap();
        let frame = Frame::Cq { sender: w1aw, grid4: fn31, tx_pwr: 10, reserved: 0 };
        let tones = encode_tones(frame);

        let sigma = 1_200.0;
        let p_n_2500 = sigma * sigma * (2500.0 / 12000.0);
        let mut rng = Rng(0xACE1_2468_1357_9BDF);
        let mut rx = Receiver::new();

        // Average estimated SNR across 8 realizations at -14, -10, -5, and +3 dB (referenced to 2,500 Hz).
        // Verify monotonic scaling and accuracy within +/- 3.0 dB (never saturating at +20 dB on weak signals).
        for &want_db in &[-14i32, -10, -5, 3] {
            let amp = (2.0 * p_n_2500 * libm_pow10(want_db as f64 / 10.0)).sqrt() as i16;
            let mut sum_db = 0i32;
            let mut cnt = 0i32;
            for _ in 0..8 {
                let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
                for s in &mut pcm {
                    *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
                }
                let start_idx = (400 * INPUT_RATE_HZ as usize) / 1000;
                let mut m = Modulator::new(tones, INPUT_RATE_HZ, 1050, DEFAULT_SPACING_HZ, amp);
                let mut idx = start_idx;
                while let Some(v) = m.next_sample() {
                    pcm[idx] = pcm[idx].saturating_add(v);
                    idx += 1;
                }
                rx.reset();
                for chunk in pcm.chunks(192) {
                    rx.push_samples(chunk);
                }
                let mut out = [Decoded { frame, audio_hz: 0, time_offset_ms: 0, snr_db: 0 }; MAX_DECODED];
                let n = rx.finish_slot(&mut out);
                if n > 0 && out[0].frame == frame {
                    sum_db += out[0].snr_db as i32;
                    cnt += 1;
                }
            }
            assert!(cnt >= 6, "expected decodes at {want_db} dB, got {cnt}/8");
            let mean_db = sum_db as f64 / cnt as f64;
            assert!(
                (mean_db - want_db as f64).abs() <= 3.0,
                "SNR readout {mean_db:.1} dB out of +/-3 dB tolerance for injected {want_db} dB"
            );
        }
    }

    #[test]
    fn swept_carrier_drift_uses_s2_branch() {
        use crate::phy::data_sym_pos;
        use crate::rx::{drift_step_half_bins, DRIFT_START_DATA};

        // 1. Verify exact per-symbol drift trajectory transitions across all 92 data symbols:
        //    - d in 0..36  (rel_sym  6..48): 0 half-bins
        //    - d in 36..71 (rel_sym 48..83): 1 half-bin  (35 intermediate-step symbols)
        //    - d in 71..92 (rel_sym 83..104): 2 half-bins (21 end-of-burst symbols)
        assert_eq!(DRIFT_START_DATA, 36);
        for d in 0..36 {
            assert_eq!(drift_step_half_bins(data_sym_pos(d)), 0, "d={d}");
        }
        for d in 36..71 {
            assert_eq!(drift_step_half_bins(data_sym_pos(d)), 1, "d={d} must use 1-half-bin intermediate step");
        }
        for d in 71..92 {
            assert_eq!(drift_step_half_bins(data_sym_pos(d)), 2, "d={d} must use 2-half-bin final step");
        }

        let k1abc = Callsign::encode(b"K1ABC").unwrap();
        let w1aw = Callsign::encode(b"NQ6N").unwrap();
        let frame = Frame::Rx { sender: k1abc, recipient: w1aw, snr_db: -12, ack: false, reserved: false };
        let tones = encode_tones(frame);

        // 2. Finite-SNR (-8 dB) +11.5 Hz and -11.5 Hz sweeps: verify S2 strictly prefers the matching drift sign
        //    AND that both the 1-half-bin (d in 36..71) and 2-half-bin (d in 71..92) segments capture distinct
        //    non-saturated shifted metrics relative to zero-drift.
        let mut rng_clean = Rng(0x2468_ACE0_1357_9BDF);
        for &(drift_mhz, expect_branch) in &[(11_500i64, 2usize), (-11_500i64, 0usize)] {
            let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
            for s in &mut pcm {
                *s = rng_clean.gauss(800.0).clamp(-32_000.0, 32_000.0) as i16;
            }
            let start_idx = (400 * INPUT_RATE_HZ as usize) / 1000;
            let mut m = Modulator::with_drift_mhz(tones, INPUT_RATE_HZ, 1275, DEFAULT_SPACING_HZ, drift_mhz, 220);
            let mut idx = start_idx;
            while let Some(v) = m.next_sample() {
                pcm[idx] = pcm[idx].saturating_add(v);
                idx += 1;
            }
            let mut rx = Receiver::new();
            for chunk in pcm.chunks(192) {
                rx.push_samples(chunk);
            }
            let (s2, diff1, diff2) = rx.top_track_drift_info().expect("track acquired");
            assert!(diff1 > 0, "1-half-bin segment (d=36..71) must diverge from zero-drift metrics");
            assert!(diff2 > 0, "2-half-bin segment (d=71..92) must diverge from zero-drift metrics");
            if expect_branch == 2 {
                assert!(s2[2] > s2[1] && s2[2] > s2[0], "+11.5 Hz sweep must score highest at +2 half-bins: {s2:?}");
            } else {
                assert!(s2[0] > s2[1] && s2[0] > s2[2], "-11.5 Hz sweep must score highest at -2 half-bins: {s2:?}");
            }
        }

        // 3. Noisy (-13 dB SNR_2500) +11.5 Hz and -11.5 Hz swept-carrier payload recovery.
        let sigma = 1_000.0;
        let p_n_2500 = sigma * sigma * (2500.0 / 12000.0);
        let amp = (2.0 * p_n_2500 * libm_pow10(-13.0 / 10.0)).sqrt() as i16;
        let mut rng = Rng(0x55AA_7733_9911_CCEE);
        let mut rx = Receiver::new();

        for &drift_mhz in &[11_500i64, -11_500i64] {
            let mut ok = 0usize;
            for _ in 0..8 {
                let mut pcm = std::vec![0i16; INPUT_RATE_HZ as usize * 10];
                for s in &mut pcm {
                    *s = rng.gauss(sigma).clamp(-32_000.0, 32_000.0) as i16;
                }
                let start_idx = ((400 + 80) * INPUT_RATE_HZ as usize) / 1000;
                let mut m = Modulator::with_drift_mhz(tones, INPUT_RATE_HZ, 1275, DEFAULT_SPACING_HZ, drift_mhz, amp);
                let mut idx = start_idx;
                while let Some(v) = m.next_sample() {
                    pcm[idx] = pcm[idx].saturating_add(v);
                    idx += 1;
                }
                rx.reset();
                for chunk in pcm.chunks(192) {
                    rx.push_samples(chunk);
                }
                let mut out = [Decoded { frame, audio_hz: 0, time_offset_ms: 0, snr_db: 0 }; MAX_DECODED];
                let n = rx.finish_slot(&mut out);
                if out[..n].iter().any(|d| d.frame == frame) {
                    ok += 1;
                }
            }
            assert!(ok >= 7, "swept carrier ({drift_mhz} mHz) decode rate too low: {ok}/8");
        }
    }

    #[test]
    fn golden_spec_test_vectors() {
        let nq6n = Callsign::encode(b"NQ6N").unwrap();
        let k1abc = Callsign::encode(b"K1ABC").unwrap();
        let en61 = encode_grid4(b"EN61").unwrap();

        let cases = [
            (
                Frame::Cq { sender: nq6n, grid4: en61, tx_pwr: 11, reserved: 0 },
                0x30DDC102171B0000u64,
                0x28CCu16,
                "30DDC102171B0000A330AACDF5B13048236A473BC10846",
                "03013201020002310130320102223001300302032210012013201100032100130122210200002200030300100130332133331200310230",
            ),
            (
                Frame::Rx { sender: nq6n, recipient: k1abc, snr_db: -12, ack: true, reserved: false },
                0xB0DDC1000F2C50B2u64,
                0x2574u16,
                "B0DDC1000F2C50B295D059FB32E01537E56358A0C06E80",
                "03013231001001021130320112103301303131110303001033201302313312123031210303113301223010100121202231100313310230",
            ),
        ];

        for (frame, exp_p64, exp_crc, exp_cw, exp_tones) in cases {
            let p64 = frame.pack_u64();
            let crc = crc14_u64(p64);
            let mut msg = [0u8; ldpc::K_BITS];
            for i in 0..64 {
                msg[i] = ((p64 >> (63 - i)) & 1) as u8;
            }
            for i in 0..14 {
                msg[64 + i] = ((crc >> (13 - i)) & 1) as u8;
            }
            let cw = ldpc::encode(&msg);
            let mut cw_hex = std::string::String::new();
            for chunk in cw.chunks(4) {
                let nib = (chunk[0] << 3) | (chunk[1] << 2) | (chunk[2] << 1) | chunk[3];
                cw_hex.push(std::char::from_digit(nib as u32, 16).unwrap().to_ascii_uppercase());
            }
            let tones = encode_tones(frame);
            let tone_str: std::string::String = tones.iter().map(|&t| (b'0' + t) as char).collect();
            assert_eq!(p64, exp_p64);
            assert_eq!(crc, exp_crc);
            assert_eq!(cw_hex, exp_cw);
            assert_eq!(tone_str, exp_tones);
            assert_eq!(Frame::unpack_u64(p64), Some(frame));
        }
    }

    fn libm_pow10(x: f64) -> f64 {
        (x * core::f64::consts::LN_10).exp()
    }
}
