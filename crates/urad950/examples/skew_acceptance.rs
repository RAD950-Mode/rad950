//! Full paired threshold acceptance suite for RAD950-60 Layout B dual-phase (`CompactReceiver60`)
//! against the legacy single-phase 13 KiB receiver (`SinglePhaseCompact60`).
//!
//! Usage:
//!   cargo run --release -p urad950 --example skew_acceptance -- [trials=200]

use std::thread;
use urad950::{
    encode_tones, frame::encode_grid4, rx::INPUT_RATE_HZ, Callsign, CompactReceiver60, Decoded,
    Frame, Modulator, SinglePhaseCompact60, DEFAULT_SPACING_HZ, MAX_DECODED,
};

const RATE: u32 = INPUT_RATE_HZ;
const REPEATS: usize = 5;
const BLOCK: usize = 192;
const SIGMA: f64 = 2_000.0;

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

fn cq(call: &[u8], grid: &[u8; 4]) -> Frame {
    Frame::Cq {
        sender: Callsign::encode(call).unwrap(),
        grid4: encode_grid4(grid).unwrap(),
        tx_pwr: 11,
        reserved: 0,
    }
}

fn amp_for_snr(db: f64) -> i16 {
    let p_n = SIGMA * SIGMA * 2500.0 / (RATE as f64 / 2.0);
    (2.0 * p_n * 10.0f64.powf(db / 10.0))
        .sqrt()
        .round()
        .clamp(0.0, 28_000.0) as i16
}

fn synth_wave(frame: Frame, hz: u16, db: f64, abs_ms: usize) -> Vec<i16> {
    let tones = encode_tones(frame);
    let mut pcm = vec![0i16; RATE as usize * 10];
    let mut m = Modulator::new(tones, RATE, u32::from(hz), DEFAULT_SPACING_HZ, amp_for_snr(db));
    let mut idx = (abs_ms * RATE as usize) / 1000;
    while let Some(v) = m.next_sample() {
        if idx < pcm.len() {
            pcm[idx] = v;
        }
        idx += 1;
    }
    pcm
}

fn noise_minute(seed: u64) -> Vec<i16> {
    let mut rng = Rng(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ 0xA5A5_5A5A);
    let mut out = vec![0i16; RATE as usize * 10 * REPEATS];
    for s in &mut out {
        *s = rng.gauss(SIGMA).clamp(-30_000.0, 30_000.0) as i16;
    }
    out
}

/// Linearly interpolate the 50% decode threshold (in dB) from `(snr_db, rate)` sorted ascending by SNR.
fn threshold_50(pts: &[(f64, f64)]) -> f64 {
    if pts[0].1 >= 0.5 {
        return pts[0].0;
    }
    for w in pts.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        if y0 <= 0.5 && y1 >= 0.5 {
            if (y1 - y0).abs() < 1e-9 {
                return 0.5 * (x0 + x1);
            }
            return x0 + (0.5 - y0) * (x1 - x0) / (y1 - y0);
        }
    }
    pts.last().unwrap().0
}

fn eval_paired(
    noises: &[Vec<i16>],
    target: Frame,
    target_wave: &[i16],
    nbr_wave: Option<&[i16]>,
    nt: usize,
) -> (usize, usize) {
    let trials = noises.len();
    let chunk = trials.div_ceil(nt);
    let mut old_ok = 0usize;
    let mut new_ok = 0usize;
    thread::scope(|s| {
        let mut hs = Vec::new();
        for ti in 0..nt {
            let st = ti * chunk;
            let en = (st + chunk).min(trials);
            if st >= en {
                continue;
            }
            hs.push(s.spawn(move || {
                let mut o_ok = 0usize;
                let mut n_ok = 0usize;
                let mut rx_old = SinglePhaseCompact60::new();
                let mut rx_new = CompactReceiver60::new();
                for i in st..en {
                    rx_old.reset();
                    rx_new.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx_old.start_subslot();
                            rx_new.start_subslot();
                        }
                        let base = sub * target_wave.len();
                        for blk in 0..(target_wave.len() / BLOCK) {
                            let o = blk * BLOCK;
                            let samp: [i16; BLOCK] = core::array::from_fn(|k| {
                                let v = noises[i][base + o + k]
                                    .saturating_add(target_wave[o + k]);
                                if let Some(nw) = nbr_wave {
                                    v.saturating_add(nw[o + k])
                                } else {
                                    v
                                }
                            });
                            rx_old.push_samples(&samp);
                            rx_new.push_samples(&samp);
                        }
                    }
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    let n = rx_old.finish_minute(&mut out);
                    if out[..n].iter().any(|d| d.frame == target) {
                        o_ok += 1;
                    }
                    let n = rx_new.finish_minute(&mut out);
                    if out[..n].iter().any(|d| d.frame == target) {
                        n_ok += 1;
                    }
                }
                (o_ok, n_ok)
            }));
        }
        for h in hs {
            let (o, n) = h.join().unwrap();
            old_ok += o;
            new_ok += n;
        }
    });
    (old_ok, new_ok)
}

fn eval_layoutb_only(
    noises: &[Vec<i16>],
    target: Frame,
    target_wave: &[i16],
    nbr_wave: Option<&[i16]>,
    nt: usize,
) -> usize {
    let trials = noises.len();
    let chunk = trials.div_ceil(nt);
    let mut new_ok = 0usize;
    thread::scope(|s| {
        let mut hs = Vec::new();
        for ti in 0..nt {
            let st = ti * chunk;
            let en = (st + chunk).min(trials);
            if st >= en {
                continue;
            }
            hs.push(s.spawn(move || {
                let mut n_ok = 0usize;
                let mut rx_new = CompactReceiver60::new();
                for i in st..en {
                    rx_new.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx_new.start_subslot();
                        }
                        let base = sub * target_wave.len();
                        for blk in 0..(target_wave.len() / BLOCK) {
                            let o = blk * BLOCK;
                            let samp: [i16; BLOCK] = core::array::from_fn(|k| {
                                let v = noises[i][base + o + k]
                                    .saturating_add(target_wave[o + k]);
                                if let Some(nw) = nbr_wave {
                                    v.saturating_add(nw[o + k])
                                } else {
                                    v
                                }
                            });
                            rx_new.push_samples(&samp);
                        }
                    }
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    let n = rx_new.finish_minute(&mut out);
                    if out[..n].iter().any(|d| d.frame == target) {
                        n_ok += 1;
                    }
                }
                n_ok
            }));
        }
        for h in hs {
            new_ok += h.join().unwrap();
        }
    });
    new_ok
}

fn eval_target_and_nbr(
    noises: &[Vec<i16>],
    target: Frame,
    nbr: Frame,
    target_wave: &[i16],
    nbr_wave: &[i16],
    nt: usize,
) -> (usize, usize) {
    let trials = noises.len();
    let chunk = trials.div_ceil(nt);
    let mut tgt_ok = 0usize;
    let mut nbr_ok = 0usize;
    thread::scope(|s| {
        let mut hs = Vec::new();
        for ti in 0..nt {
            let st = ti * chunk;
            let en = (st + chunk).min(trials);
            if st >= en {
                continue;
            }
            hs.push(s.spawn(move || {
                let (mut t_ok, mut n_ok) = (0usize, 0usize);
                let mut rx = CompactReceiver60::new();
                for i in st..en {
                    rx.reset();
                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx.start_subslot();
                        }
                        let base = sub * target_wave.len();
                        for blk in 0..(target_wave.len() / BLOCK) {
                            let o = blk * BLOCK;
                            let samp: [i16; BLOCK] = core::array::from_fn(|k| {
                                noises[i][base + o + k]
                                    .saturating_add(target_wave[o + k])
                                    .saturating_add(nbr_wave[o + k])
                            });
                            rx.push_samples(&samp);
                        }
                    }
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    let n = rx.finish_minute(&mut out);
                    if out[..n].iter().any(|d| d.frame == target) {
                        t_ok += 1;
                    }
                    if out[..n].iter().any(|d| d.frame == nbr) {
                        n_ok += 1;
                    }
                }
                (t_ok, n_ok)
            }));
        }
        for h in hs {
            let (t, n) = h.join().unwrap();
            tgt_ok += t;
            nbr_ok += n;
        }
    });
    (tgt_ok, nbr_ok)
}

fn main() {
    let arg1 = std::env::args().nth(1);
    let nt = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    let target = cq(b"NQ6N", b"EN61");
    let nbr = cq(b"WP2Z", b"FN31");

    if arg1.as_deref() == Some("nbr225") {
        let trials: usize = std::env::args()
            .nth(2)
            .and_then(|s| s.parse().ok())
            .unwrap_or(200);
        let noises: Vec<Vec<i16>> = (0..trials)
            .map(|i| noise_minute(8_500_000 + i as u64))
            .collect();
        let snrs = [-21.5, -21.0, -20.5, -20.0, -19.5, -19.0, -18.5];
        println!("=== ADJACENT 225 Hz +18 dB NEIGHBOR (1950 Hz target, 2175 Hz +18 dB nbr, {trials} trials/pt) ===");
        println!("case                       | tgt_50%  | nbr_ok @-20dB");
        println!("---------------------------+----------+--------------");
        for (label, t_off, n_off) in [
            ("tgt 0ms,   nbr 0ms", 400usize, 400usize),
            ("tgt +40ms, nbr 0ms", 440, 400),
            ("tgt +40ms, nbr +40ms", 440, 440),
            ("tgt +20ms, nbr 0ms", 420, 400),
            ("tgt -40ms, nbr 0ms", 360, 400),
        ] {
            let mut pts = Vec::new();
            let mut nbr_at_20 = 0usize;
            for &db in &snrs {
                let tw = synth_wave(target, 1950, db, t_off);
                let nw = synth_wave(nbr, 2175, db + 18.0, n_off);
                let (t_ok, n_ok) = eval_target_and_nbr(&noises, target, nbr, &tw, &nw, nt);
                pts.push((db, t_ok as f64 / trials as f64));
                if (db + 20.0).abs() < 0.01 {
                    nbr_at_20 = n_ok;
                }
            }
            let thr = threshold_50(&pts);
            println!("{label:<26} | {thr:>6.2}dB | {nbr_at_20:>3}/{trials}");
        }
        return;
    }

    let trials: usize = arg1.and_then(|s| s.parse().ok()).unwrap_or(200);

    println!("Generating {trials} shared noise minutes (seed base 8_000_000)...");
    let noises: Vec<Vec<i16>> = (0..trials)
        .map(|i| noise_minute(8_000_000 + i as u64))
        .collect();

    // Part A: Aligned (0 ms) paired regression vs legacy SinglePhaseCompact60 (gate: <= +0.2 dB)
    let snrs_aligned = [-22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0, -18.5];
    println!("\n=== PART A: ALIGNED (0 ms) PAIRED THRESHOLD vs LEGACY SINGLE-PHASE ({trials} trials/pt) ===");
    println!("condition              | old_50%  | new_50%  | delta_dB | gate(<=+0.2)");
    println!("-----------------------+----------+----------+----------+-------------");
    let mut aligned_1275_iso = 0.0;
    let mut aligned_2175_nbr = 0.0;
    let aligned_cases: [(&str, u16, Option<u16>); 4] = [
        ("1275 Hz isolated", 1275, None),
        ("2175 Hz isolated", 2175, None),
        ("1275 Hz +18dB @2175", 1275, Some(2175)),
        ("2175 Hz +18dB @1275", 2175, Some(1275)),
    ];
    for (name, hz, nbr_hz) in aligned_cases {
        let mut old_pts = Vec::new();
        let mut new_pts = Vec::new();
        for &db in &snrs_aligned {
            let tw = synth_wave(target, hz, db, 400);
            let nw = nbr_hz.map(|nhz| synth_wave(nbr, nhz, db + 18.0, 400));
            let (o, n) = eval_paired(&noises, target, &tw, nw.as_deref(), nt);
            old_pts.push((db, o as f64 / trials as f64));
            new_pts.push((db, n as f64 / trials as f64));
        }
        let t_old = threshold_50(&old_pts);
        let t_new = threshold_50(&new_pts);
        let delta = t_new - t_old;
        if name == "1275 Hz isolated" {
            aligned_1275_iso = t_new;
        }
        if name == "2175 Hz +18dB @1275" {
            aligned_2175_nbr = t_new;
        }
        println!(
            "{name:<22} | {t_old:>6.2}dB | {t_new:>6.2}dB | {delta:>+7.2}dB | {}",
            if delta <= 0.2 { "PASS" } else { "FAIL" }
        );
    }

    // Part B1: Full skew sweep (-200..+200 ms, 10 ms steps, 41 offsets) isolated 1275 Hz (gate: <= +1.5 dB)
    let snrs_skew = [-21.5, -21.0, -20.5, -20.0, -19.5, -19.0, -18.5];
    let offsets: Vec<i32> = (-200..=200).step_by(10).collect();
    println!("\n=== PART B1: SKEW SWEEP (-200..+200 ms, 10 ms steps, 1275 Hz isolated, {trials} trials/pt) ===");
    println!("offset_ms | thr_50%  | loss_vs_0ms | gate(<=+1.5)");
    println!("----------+----------+-------------+-------------");
    let mut worst_iso = (0i32, -99.0f64);
    for &off in &offsets {
        let abs_ms = (400 + off) as usize;
        let mut pts = Vec::new();
        for &db in &snrs_skew {
            let tw = synth_wave(target, 1275, db, abs_ms);
            let n = eval_layoutb_only(&noises, target, &tw, None, nt);
            pts.push((db, n as f64 / trials as f64));
        }
        let thr = threshold_50(&pts);
        let loss = thr - aligned_1275_iso;
        if loss > worst_iso.1 {
            worst_iso = (off, loss);
        }
        println!(
            "{off:>+8}  | {thr:>6.2}dB | {loss:>+9.2}dB | {}",
            if loss <= 1.5 { "PASS" } else { "FAIL" }
        );
    }
    println!(
        "WORST ISOLATED SKEW: {:+} ms -> {:+.2} dB (gate <= +1.50 dB: {})",
        worst_iso.0,
        worst_iso.1,
        if worst_iso.1 <= 1.5 { "PASS" } else { "FAIL" }
    );

    // Part B2: Key skew offsets under +18 dB neighbor (2175 Hz target, 1275 Hz +18 dB neighbor @ 0 ms)
    let nbr_offsets = [-200, -160, -120, -80, -40, -20, 0, 20, 40, 60, 80, 120, 160, 200];
    let snrs_nbr_skew = [-21.0, -20.5, -20.0, -19.5, -19.0, -18.5, -18.0, -17.5, -17.0];
    println!("\n=== PART B2: SKEW +18 dB NEIGHBOR (2175 Hz target, 1275 Hz +18dB nbr, {trials} trials/pt) ===");
    println!("offset_ms | thr_50%  | loss_vs_0ms");
    println!("----------+----------+------------");
    let mut worst_nbr = (0i32, -99.0f64);
    for &off in &nbr_offsets {
        let abs_ms = (400 + off) as usize;
        let mut pts = Vec::new();
        for &db in &snrs_nbr_skew {
            let tw = synth_wave(target, 2175, db, abs_ms);
            let nw = synth_wave(nbr, 1275, db + 18.0, 400);
            let n = eval_layoutb_only(&noises, target, &tw, Some(&nw), nt);
            pts.push((db, n as f64 / trials as f64));
        }
        let thr = threshold_50(&pts);
        let loss = thr - aligned_2175_nbr;
        if loss > worst_nbr.1 {
            worst_nbr = (off, loss);
        }
        println!("{off:>+8}  | {thr:>6.2}dB | {loss:>+9.2}dB");
    }
    println!(
        "WORST +18 dB NEIGHBOR SKEW: {:+} ms -> {:+.2} dB",
        worst_nbr.0, worst_nbr.1
    );
}
