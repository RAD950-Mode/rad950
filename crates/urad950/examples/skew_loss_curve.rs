//! Clock-skew proposal Step 1 (first bullet):
//! Sweep sender timing offset from -200 ms to +200 ms in 5 ms steps at three SNRs
//! (-19, -20, -21 dB) on `CompactReceiver60`, mapping the half-symbol dead zone,
//! shoulders, and late-side (+80 ms) search cutoff.
//!
//! Usage:
//!   cargo run --release --example skew_loss_curve -- [trials_per_point]

use std::thread;
use urad950::{
    encode_tones, Callsign, CompactReceiver60, Decoded, Frame, Modulator, DEFAULT_SPACING_HZ,
    MAX_DECODED,
};

const RATE: u32 = 24_000;
const SIGMA: f64 = 1_500.0;
const REPEATS: usize = 5;
const BLOCK: usize = 192;

fn cq(call: &[u8]) -> Frame {
    Frame::Cq {
        sender: Callsign::encode(call).unwrap(),
        grid4: 1234,
        tx_pwr: 10,
        reserved: 0,
    }
}

fn synth_wave(frame: Frame, hz: u32, db: f64, abs_start_ms: usize) -> Vec<i16> {
    let np = SIGMA.powi(2) * 2500.0 / (RATE / 2) as f64;
    let amp = (2.0 * np * 10f64.powf(db / 10.0)).sqrt().round().max(1.0) as i16;
    let mut wave = vec![0i16; RATE as usize * 10];
    let mut tx = Modulator::new(encode_tones(frame), RATE, hz, DEFAULT_SPACING_HZ, amp);
    let start_idx = abs_start_ms * (RATE as usize) / 1000;
    for s in &mut wave[start_idx..] {
        let Some(v) = tx.next_sample() else { break };
        *s = v;
    }
    wave
}

fn noise_minute(seed: u64) -> Vec<i16> {
    let mut st = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ 0xA5A5_5A5A_1234_5678;
    if st == 0 {
        st = 1;
    }
    (0..RATE as usize * 10 * REPEATS)
        .map(|_| {
            let mut s = -6.0;
            for _ in 0..12 {
                st ^= st << 13;
                st ^= st >> 7;
                st ^= st << 17;
                s += (st >> 16) as u32 as f64 / u32::MAX as f64;
            }
            (s * SIGMA).round().clamp(-32_000.0, 32_000.0) as i16
        })
        .collect()
}

fn main() {
    let trials: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(64);
    let target = cq(b"NQ6N");
    let snrs = [-19.0f64, -20.0, -21.0];
    let offsets: Vec<i32> = (-200..=200).step_by(5).collect();
    let nt = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    println!("=== RAD950-60 CLOCK-SKEW LOSS CURVE (CompactReceiver60, 1275 Hz, {trials} trials/pt) ===");
    println!("offset_ms | abs_ms |  -19 dB |  -20 dB |  -21 dB");
    println!("----------+--------+---------+---------+--------");

    // Pre-generate `trials` noise realizations shared across offsets for clean paired comparison
    let noises: Vec<Vec<i16>> = (0..trials)
        .map(|i| noise_minute(7_000_000 + i as u64))
        .collect();

    for &off_ms in &offsets {
        let abs_ms = (400 + off_ms) as usize;
        let mut counts = [0usize; 3];
        for (si, &db) in snrs.iter().enumerate() {
            let wave = synth_wave(target, 1275, db, abs_ms);
            let chunk = trials.div_ceil(nt);
            thread::scope(|s| {
                let mut hs = Vec::new();
                for ti in 0..nt {
                    let st = ti * chunk;
                    let en = (st + chunk).min(trials);
                    if st >= en {
                        continue;
                    }
                    let w_ref = &wave;
                    let n_ref = &noises;
                    hs.push(s.spawn(move || {
                        let mut ok = 0usize;
                        let mut rx = CompactReceiver60::new();
                        for i in st..en {
                            rx.reset();
                            for sub in 0..REPEATS {
                                if sub > 0 {
                                    rx.start_subslot();
                                }
                                let base = sub * w_ref.len();
                                for blk in 0..(w_ref.len() / BLOCK) {
                                    let o = blk * BLOCK;
                                    let samp: [i16; BLOCK] = core::array::from_fn(|k| {
                                        n_ref[i][base + o + k].saturating_add(w_ref[o + k])
                                    });
                                    rx.push_samples(&samp);
                                }
                            }
                            let mut out = [Decoded::EMPTY; MAX_DECODED];
                            let n = rx.finish_minute(&mut out);
                            if out[..n].iter().any(|d| d.frame == target) {
                                ok += 1;
                            }
                        }
                        ok
                    }));
                }
                for h in hs {
                    counts[si] += h.join().unwrap();
                }
            });
        }
        println!(
            "{:>+8}  | {:>4}ms | {:>5.1}%  | {:>5.1}%  | {:>5.1}%",
            off_ms,
            abs_ms,
            100.0 * counts[0] as f64 / trials as f64,
            100.0 * counts[1] as f64 / trials as f64,
            100.0 * counts[2] as f64 / trials as f64,
        );
    }
}
