//! Step 2 host sensitivity study: floating-point reference ablation ladder +
//! replay-only oracle miss attribution for RAD950-60.
//!
//! Usage:
//!   cargo run --release --example sensitivity_study -- ladder [trials_per_snr]
//!   cargo run --release --example sensitivity_study -- oracle [trials_per_cond]
//!   cargo run --release --example sensitivity_study -- all [ladder_trials] [oracle_trials]

use std::f64::consts::TAU;
use std::sync::OnceLock;
use std::thread;
use urad950::dsp::{cfft, log2_q8, lowpass, real_power, Decimator};
use urad950::ldpc::{self, BpScratch, K_BITS, N_BITS};
use urad950::phy::{data_sym_pos, interleave_idx, N_DATA_SYMS};
use urad950::rx::{FFT_M, FFT_N, FIRST_BIN, N_BINS, SYM_SAMPLES};
use urad950::{
    encode_tones, Callsign, CompactReceiver60, Decoded, Frame, LossStage, Modulator,
    OracleAttribution, OracleProbe, Receiver60, DEFAULT_SPACING_HZ, MAX_DECODED,
};

const RATE: u32 = 24_000;
const DECIM_RATE: f64 = 6_000.0;
const SIGMA: f64 = 1_500.0;
const REPEATS: usize = 5;
const BLOCK: usize = 192;
const RAMP_SAMPLES_6K: usize = 60;

static LP_24K_TO_6K: [i16; 31] = lowpass::<31>(2_550.0 / 24_000.0);

/// Systematic `(78 x 184)` generator matrix over GF(2), packed into `[u64; 3]` per row.
static GEN_MATRIX: OnceLock<[[u64; 3]; K_BITS]> = OnceLock::new();

fn gen_matrix() -> &'static [[u64; 3]; K_BITS] {
    GEN_MATRIX.get_or_init(|| {
        let mut g = [[0u64; 3]; K_BITS];
        for r in 0..K_BITS {
            let mut msg = [0u8; K_BITS];
            msg[r] = 1;
            let cw = ldpc::encode(&msg);
            for (c, &bit) in cw.iter().enumerate() {
                if bit != 0 {
                    g[r][c >> 6] |= 1u64 << (c & 63);
                }
            }
        }
        g
    })
}

#[inline]
fn get_bit(row: &[u64; 3], col: usize) -> u8 {
    ((row[col >> 6] >> (col & 63)) & 1) as u8
}

#[inline]
fn xor_row(dst: &mut [u64; 3], src: &[u64; 3]) {
    dst[0] ^= src[0];
    dst[1] ^= src[1];
    dst[2] ^= src[2];
}

/// Ordered-Statistics Decoding (order 1) on `(184, 78)` using the same LLRs as BP.
/// Note: sign convention in `urad950::ldpc`: `llr < 0 => bit = 1`, `llr >= 0 => bit = 0`.
fn decode_osd1(llr: &[f64; N_BITS]) -> Option<Frame> {
    let g = gen_matrix();

    // Sort columns by descending reliability |llr|.
    let mut order: [usize; N_BITS] = core::array::from_fn(|i| i);
    order.sort_unstable_by(|&a, &b| llr[b].abs().total_cmp(&llr[a].abs()));

    // Copy G and perform Gaussian elimination in `order` to find K_BITS independent columns (MRB).
    let mut mat = *g;
    let mut mrb_cols = [0usize; K_BITS];
    let mut pivot_row = 0usize;
    for &col in &order {
        if pivot_row == K_BITS {
            break;
        }
        let mut found = None;
        for r in pivot_row..K_BITS {
            if get_bit(&mat[r], col) != 0 {
                found = Some(r);
                break;
            }
        }
        let Some(pr) = found else { continue };
        mat.swap(pivot_row, pr);
        let prow = mat[pivot_row];
        for r in 0..K_BITS {
            if r != pivot_row && get_bit(&mat[r], col) != 0 {
                xor_row(&mut mat[r], &prow);
            }
        }
        mrb_cols[pivot_row] = col;
        pivot_row += 1;
    }
    if pivot_row != K_BITS {
        return None;
    }

    // Hard decisions for all bits: bit = 1 iff llr < 0.
    let mut hd = [0u64; 3];
    for (j, &v) in llr.iter().enumerate() {
        if v < 0.0 {
            hd[j >> 6] |= 1u64 << (j & 63);
        }
    }

    // Order-0 candidate codeword from MRB hard decisions.
    let mut cw0 = [0u64; 3];
    for r in 0..K_BITS {
        if get_bit(&hd, mrb_cols[r]) != 0 {
            xor_row(&mut cw0, &mat[r]);
        }
    }

    let check_candidate = |cw: &[u64; 3]| -> Option<(f64, Frame)> {
        let mut bits78 = [0u8; K_BITS];
        for i in 0..K_BITS {
            bits78[i] = get_bit(cw, i);
        }
        let frame = Frame::from_bits78(&bits78)?;
        let mut cost = 0.0f64;
        for w in 0..3 {
            let mut diff = cw[w] ^ hd[w];
            while diff != 0 {
                let tz = diff.trailing_zeros() as usize;
                let idx = (w << 6) + tz;
                if idx < N_BITS {
                    cost += llr[idx].abs();
                }
                diff &= diff - 1;
            }
        }
        Some((cost, frame))
    };

    let mut best: Option<(f64, Frame)> = check_candidate(&cw0);

    // Order-1: flip each single MRB bit.
    for r in 0..K_BITS {
        let mut cw = cw0;
        xor_row(&mut cw, &mat[r]);
        if let Some((cost, frame)) = check_candidate(&cw) {
            if best.map_or(true, |(bc, _)| cost < bc) {
                best = Some((cost, frame));
            }
        }
    }
    best.map(|(_, f)| f)
}

fn cq(call: &[u8], grid: u16) -> Frame {
    Frame::Cq {
        sender: Callsign::encode(call).unwrap(),
        grid4: grid,
        tx_pwr: 10,
        reserved: 0,
    }
}

fn snr_to_amp(db: f64) -> i16 {
    let noise_power = SIGMA.powi(2) * 2500.0 / (RATE / 2) as f64;
    (2.0 * noise_power * 10f64.powf(db / 10.0))
        .sqrt()
        .round()
        .max(1.0) as i16
}

fn synth_subslot_wave(frame: Frame, hz: u32, db: f64, offset_ms: usize) -> Vec<i16> {
    let amp = snr_to_amp(db);
    let mut wave = vec![0i16; RATE as usize * 10];
    let mut tx = Modulator::new(encode_tones(frame), RATE, hz, DEFAULT_SPACING_HZ, amp);
    for sample in &mut wave[offset_ms * 24..] {
        let Some(v) = tx.next_sample() else { break };
        *sample = v;
    }
    wave
}

fn synth_noise_minute(seed: u64) -> Vec<i16> {
    let mut state = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ 0xA5A5_5A5A_1234_5678;
    if state == 0 {
        state = 1;
    }
    (0..RATE as usize * 10 * REPEATS)
        .map(|_| {
            let mut sum = -6.0;
            for _ in 0..12 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                sum += (state >> 16) as u32 as f64 / u32::MAX as f64;
            }
            (sum * SIGMA).round().clamp(-32_000.0, 32_000.0) as i16
        })
        .collect()
}

/// Exact DFT power at `freq_hz` over `samples` at 6 kHz.
#[inline]
fn exact_tone_power(samples: &[i16], freq_hz: f64) -> f64 {
    let w = TAU * freq_hz / DECIM_RATE;
    let cw = w.cos();
    let sw = w.sin();
    let mut cr = 1.0f64;
    let mut ci = 0.0f64;
    let mut re = 0.0f64;
    let mut im = 0.0f64;
    for &s in samples {
        let x = s as f64;
        re += x * cr;
        im -= x * ci;
        let ncr = cr * cw - ci * sw;
        let nci = cr * sw + ci * cw;
        cr = ncr;
        ci = nci;
    }
    re * re + im * im
}

/// Convert 4-tone metrics `[N_DATA_SYMS][4]` into deinterleaved float LLRs and i8 LLRs,
/// then decode with BP(35).
fn decode_bp_from_tone_metrics(metrics: &[[f64; 4]; N_DATA_SYMS]) -> (Option<Frame>, [f64; N_BITS]) {
    let mut inter = [0.0f64; N_BITS];
    for d in 0..N_DATA_SYMS {
        let m = metrics[d];
        inter[2 * d] = m[0].max(m[1]) - m[2].max(m[3]);
        inter[2 * d + 1] = m[0].max(m[3]) - m[1].max(m[2]);
    }
    let mut deinter = [0.0f64; N_BITS];
    let mut max_abs = 1e-12f64;
    for i in 0..N_BITS {
        let v = inter[interleave_idx(i)];
        deinter[i] = v;
        max_abs = max_abs.max(v.abs());
    }
    let mut llr_i8 = [0i8; N_BITS];
    for i in 0..N_BITS {
        llr_i8[i] = ((deinter[i] * 60.0) / max_abs).round().clamp(-120.0, 120.0) as i8;
    }
    let mut scratch = BpScratch::new();
    let frame = ldpc::decode_bp(&llr_i8, &mut scratch, 35).and_then(|b| Frame::from_bits78(&b));
    (frame, deinter)
}

#[derive(Clone, Copy, Default)]
struct LadderOutcome {
    fp_lin_80ms: bool,
    fp_log_80ms: bool,
    q6_80ms_exact: bool,
    q6_70ms_exact: bool,
    hh_prim_oracle: bool,
    hh_prim_bp_osd1: bool,
    hh_prim_blind: bool,
    hh_compact_blind: bool,
    desktop_dual_blind: bool,
}

const LADDER_NAMES: [&str; 9] = [
    "0:fp_lin_80ms_exact",
    "1:fp_log_80ms_exact",
    "2:q6_80ms_exact",
    "3:q6_70ms_exact",
    "4:hh_prim_oracle",
    "5:hh_prim_bp+osd1",
    "6:hh_prim_blind",
    "7:hh_compact_blind",
    "8:desktop_dual_blind",
];

impl LadderOutcome {
    fn as_array(&self) -> [bool; 9] {
        [
            self.fp_lin_80ms,
            self.fp_log_80ms,
            self.q6_80ms_exact,
            self.q6_70ms_exact,
            self.hh_prim_oracle,
            self.hh_prim_bp_osd1,
            self.hh_prim_blind,
            self.hh_compact_blind,
            self.desktop_dual_blind,
        ]
    }
}

/// Run all ladder stages + handheld + desktop receivers on one 5-subslot minute.
fn eval_ladder_minute(
    subslots_24k: &[Vec<i16>; REPEATS],
    target: Frame,
    target_hz: u16,
    target_offset_ms: u16,
) -> LadderOutcome {
    let start_sym = ((target_offset_ms as usize) + 40) / 80;
    let tone_freqs: [f64; 4] = core::array::from_fn(|t| target_hz as f64 + t as f64 * 50.0);

    let mut acc_lin_80 = [[0.0f64; 4]; N_DATA_SYMS];
    let mut acc_log_80 = [[0.0f64; 4]; N_DATA_SYMS];
    let mut acc_q6_80 = [[0.0f64; 4]; N_DATA_SYMS];
    let mut acc_q6_70 = [[0.0f64; 4]; N_DATA_SYMS];

    let mut rx_compact = CompactReceiver60::new();
    let mut rx_desk_peak = Receiver60::new();
    let mut rx_desk_pct = Receiver60::with_percentile_span();

    for (sub, pcm24) in subslots_24k.iter().enumerate() {
        if sub > 0 {
            rx_compact.start_subslot();
            rx_desk_peak.start_subslot();
            rx_desk_pct.start_subslot();
        }
        // Feed actual receivers
        for chunk in pcm24.chunks(BLOCK) {
            rx_compact.push_samples(chunk);
            rx_desk_peak.push_samples(chunk);
            rx_desk_pct.push_samples(chunk);
        }

        // Decimate identically to 6 kHz for the reference ladder
        let mut decim = Decimator::<31>::new();
        let mut pcm6 = Vec::with_capacity(pcm24.len() / 4);
        for &s in pcm24 {
            if let Some(y) = decim.push(s, &LP_24K_TO_6K, 4) {
                pcm6.push(y);
            }
        }

        // Evaluate every data symbol of the target
        for d in 0..N_DATA_SYMS {
            let sym = start_sym + data_sym_pos(d);
            let s0 = sym * SYM_SAMPLES;
            let s1 = s0 + SYM_SAMPLES;
            if s1 > pcm6.len() {
                continue;
            }
            let sym80 = &pcm6[s0..s1];
            let sym70 = &pcm6[s0 + RAMP_SAMPLES_6K..s1];

            // Compute exact same passband floor & peak span as rx60.rs for steps 2 and 3
            let mut z = [0i16; FFT_N];
            z[..sym70.len()].copy_from_slice(sym70);
            let shift = cfft(&mut z, FFT_M);
            let mut sum = 0u32;
            let mut peak = 0u8;
            real_power(&z, FFT_M, FIRST_BIN, 1, N_BINS, |_, pwr| {
                let lp = (log2_q8(pwr.max(1)) + ((2 * shift as i32 - 13) << 8)) >> 5;
                let v = lp.clamp(0, 255) as u8;
                sum += v as u32;
                if v > peak {
                    peak = v;
                }
            });
            let floor = (sum / N_BINS as u32) as i32;
            let span = (i32::from(peak) - floor).clamp(18, 96);

            // Convert floating-point DFT power to the same log2_q8/32 scale as `cur_bins`:
            // In cfft/real_power, `z` is scaled by 2^-shift (effective FFT gain 1/512 * 2^-shift),
            // and `real_power` returns |X|^2 / 4 (or |X|^2 for real_fft). Empirically, exact_tone_power
            // on 420 samples equals `4 * (512/2)^2 * ...` -> align scale cleanly via log2:
            // real_power returns `(re^2 + im^2)` where `cfft` divides by 2^shift on 512-pt input;
            // exact_tone_power computes `(sum x cos)^2 + (sum x sin)^2` without any 1/2^shift division.
            // In `rx60.rs`: `lp = (log2_q8(pwr) + ((2*shift - 13) << 8)) >> 5`, where `pwr = (Re^2+Im^2) / 2^(2*shift) * 2^?`
            // Wait: `2*shift - 13` corresponds to `log2(exact_power) - 13`! Let's verify:
            // In `dsp::real_power`, FFT output is `X[k] / 2^shift` -> `|X|^2 / 2^(2*shift)`.
            // Adding `2*shift << 8` cancels `2^(2*shift)`, leaving `log2(|X|^2) - 13`, then `* 8` (`<< 8 >> 5` = `<< 3`).
            // For 80 ms (480 samples) vs 70 ms (420 samples), coherent power scales by `(480/420)^2`,
            // so normalize 80 ms power by `(420/480)^2` before comparing to the 70 ms passband floor!
            let norm80 = (420.0 / 480.0) * (420.0 / 480.0);

            for t in 0..4 {
                let p80 = exact_tone_power(sym80, tone_freqs[t]).max(1.0);
                let p70 = exact_tone_power(sym70, tone_freqs[t]).max(1.0);

                acc_lin_80[d][t] += p80;
                acc_log_80[d][t] += p80.ln();

                // Map to rx60.rs `cur_bins` scale: `((log2(p) - 13.0) * 8.0).clamp(0, 255)`
                let lp80 = (((p80 * norm80).log2() - 13.0) * 8.0).clamp(0.0, 255.0) as i32;
                let q80 = ((lp80 - floor + 3).max(0) * 6 / span).clamp(0, 6);
                acc_q6_80[d][t] += q80 as f64;

                let lp70 = ((p70.log2() - 13.0) * 8.0).clamp(0.0, 255.0) as i32;
                let q70 = ((lp70 - floor + 3).max(0) * 6 / span).clamp(0, 6);
                acc_q6_70[d][t] += q70 as f64;
            }
        }
    }

    let (f_lin, _) = decode_bp_from_tone_metrics(&acc_lin_80);
    let (f_log, _) = decode_bp_from_tone_metrics(&acc_log_80);
    let (f_q80, _) = decode_bp_from_tone_metrics(&acc_q6_80);
    let (f_q70, _) = decode_bp_from_tone_metrics(&acc_q6_70);

    let probe = [OracleProbe {
        expected_frame: target,
        audio_hz: target_hz,
        offset_ms: target_offset_ms,
    }];
    let mut attr = [OracleAttribution::EMPTY];
    let mut out_c = [Decoded::EMPTY; MAX_DECODED];
    let (prim_n, comb_n) = rx_compact.finish_minute_with_oracle(&mut out_c, &probe, &mut attr);

    // Step 5: BP + OSD-1 on the exact same handheld primary oracle LLRs
    let hh_bp_osd1 = if attr[0].primary_oracle_decoded_expected {
        true
    } else {
        let llr_i8 = rx_compact.primary_llrs(attr[0].start_sym, attr[0].f0_half);
        let llr_f64: [f64; N_BITS] = core::array::from_fn(|i| llr_i8[i] as f64);
        decode_osd1(&llr_f64) == Some(target)
    };

    // Desktop dual pass
    let mut out_dp = [Decoded::EMPTY; MAX_DECODED];
    let mut out_dt = [Decoded::EMPTY; MAX_DECODED];
    let n_dp = rx_desk_peak.finish_minute(&mut out_dp);
    let n_dt = rx_desk_pct.finish_minute(&mut out_dt);
    let desktop_ok = out_dp[..n_dp].iter().any(|d| d.frame == target)
        || out_dt[..n_dt].iter().any(|d| d.frame == target);

    LadderOutcome {
        fp_lin_80ms: f_lin == Some(target),
        fp_log_80ms: f_log == Some(target),
        q6_80ms_exact: f_q80 == Some(target),
        q6_70ms_exact: f_q70 == Some(target),
        hh_prim_oracle: attr[0].primary_oracle_decoded_expected,
        hh_prim_bp_osd1: hh_bp_osd1,
        hh_prim_blind: out_c[..prim_n].iter().any(|d| d.frame == target),
        hh_compact_blind: out_c[..comb_n].iter().any(|d| d.frame == target),
        desktop_dual_blind: desktop_ok,
    }
}

/// Logistic regression `P(x) = 1 / (1 + exp(-(a + b*x)))` via IRLS.
/// Returns `Some((snr_50, slope))`.
fn fit_logistic_50(snrs: &[f64], hits: &[usize], trials: &[usize]) -> Option<(f64, f64)> {
    let mut a = 22.0f64;
    let mut b = 1.0f64;
    for _ in 0..40 {
        let mut g0 = 0.0;
        let mut g1 = 0.0;
        let mut h00 = 1e-6;
        let mut h01 = 0.0;
        let mut h11 = 1e-6;
        for ((&x, &k), &n) in snrs.iter().zip(hits).zip(trials) {
            let nf = n as f64;
            let kf = k as f64;
            let z = (a + b * x).clamp(-30.0, 30.0);
            let p = 1.0 / (1.0 + (-z).exp());
            let w = nf * p * (1.0 - p);
            g0 += kf - nf * p;
            g1 += (kf - nf * p) * x;
            h00 += w;
            h01 += w * x;
            h11 += w * x * x;
        }
        let det = h00 * h11 - h01 * h01;
        if det.abs() < 1e-12 {
            break;
        }
        let da = (h11 * g0 - h01 * g1) / det;
        let db = (-h01 * g0 + h00 * g1) / det;
        a += da;
        b += db;
        if da.abs() + db.abs() < 1e-7 {
            break;
        }
    }
    if b <= 1e-4 {
        return None;
    }
    Some((-a / b, b))
}

/// Bootstrap 95% CI for the logistic 50% point by resampling binomial counts at each SNR.
fn bootstrap_ci_50(snrs: &[f64], hits: &[usize], trials: &[usize]) -> (f64, f64) {
    let mut state = 0x1234_5678_9ABC_DEF0u64;
    let mut next_f64 = || -> f64 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / ((1u64 << 53) as f64)
    };
    let mut ests = Vec::with_capacity(400);
    let mut bhits = vec![0usize; snrs.len()];
    for _ in 0..400 {
        for i in 0..snrs.len() {
            let p = hits[i] as f64 / trials[i] as f64;
            let mut k = 0;
            for _ in 0..trials[i] {
                if next_f64() < p {
                    k += 1;
                }
            }
            bhits[i] = k;
        }
        if let Some((x50, _)) = fit_logistic_50(snrs, &bhits, trials) {
            if x50.is_finite() {
                ests.push(x50);
            }
        }
    }
    if ests.len() < 20 {
        return (f64::NAN, f64::NAN);
    }
    ests.sort_by(|a, b| a.total_cmp(b));
    (ests[ests.len() * 25 / 1000], ests[ests.len() * 975 / 1000])
}

/// Exact two-sided McNemar binomial p-value for discordant pairs `(b, c)`.
fn mcnemar_p(b: usize, c: usize) -> f64 {
    let n = b + c;
    if n == 0 {
        return 1.0;
    }
    let k = b.min(c);
    // Compute exact 2 * sum_{i=0..k} C(n, i) * 2^-n in log-space for stability.
    let mut log_fact = vec![0.0f64; n + 1];
    for i in 1..=n {
        log_fact[i] = log_fact[i - 1] + (i as f64).ln();
    }
    let ln2 = 2.0f64.ln();
    let mut sum = 0.0f64;
    for i in 0..=k {
        let lp = log_fact[n] - log_fact[i] - log_fact[n - i] - (n as f64) * ln2;
        sum += lp.exp();
    }
    (2.0 * sum).min(1.0)
}

#[derive(Clone, Copy)]
enum LadderEnv {
    Isolated { hz: u16 },
    Neighbor18 { hz: u16, nbr_hz: u16 },
    ImpulseSubslot { hz: u16 },
}

fn run_ladder_condition(name: &str, env: LadderEnv, snrs: &[f64], trials_per_snr: usize, base_seed: u64) {
    let target = cq(b"NQ6N", 1234);
    let neighbor = cq(b"K1ABC", 2345);
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);

    println!("\n=== LADDER: {name} ({trials_per_snr} trials/SNR, {} SNRs) ===", snrs.len());
    let mut hits = vec![[0usize; 9]; snrs.len()];
    // Store per-SNR per-trial outcomes so we can compute exact paired McNemar at the knee
    let mut all_outcomes: Vec<Vec<[bool; 9]>> = Vec::with_capacity(snrs.len());

    for (s_idx, &snr_db) in snrs.iter().enumerate() {
        let target_hz = match env {
            LadderEnv::Isolated { hz }
            | LadderEnv::Neighbor18 { hz, .. }
            | LadderEnv::ImpulseSubslot { hz } => hz,
        };
        let target_wave = synth_subslot_wave(target, target_hz as u32, snr_db, 400);
        let nbr_wave = match env {
            LadderEnv::Neighbor18 { nbr_hz, .. } => {
                Some(synth_subslot_wave(neighbor, nbr_hz as u32, snr_db + 18.0, 440))
            }
            _ => None,
        };

        let chunk = trials_per_snr.div_ceil(n_threads);
        let mut results: Vec<[bool; 9]> = Vec::with_capacity(trials_per_snr);

        thread::scope(|s| {
            let mut handles = Vec::new();
            for t_idx in 0..n_threads {
                let start = t_idx * chunk;
                let end = (start + chunk).min(trials_per_snr);
                if start >= end {
                    continue;
                }
                let tw = &target_wave;
                let nw = &nbr_wave;
                handles.push(s.spawn(move || {
                    let mut local = Vec::with_capacity(end - start);
                    for i in start..end {
                        let seed = base_seed
                            .wrapping_add((s_idx as u64) * 10_000)
                            .wrapping_add(i as u64);
                        let noise_min = synth_noise_minute(seed);
                        let mut subslots: [Vec<i16>; REPEATS] = core::array::from_fn(|sub| {
                            let base = sub * tw.len();
                            let mut v = vec![0i16; tw.len()];
                            for j in 0..tw.len() {
                                let mut acc = i32::from(noise_min[base + j]) + i32::from(tw[j]);
                                if let Some(nwave) = nw {
                                    acc += i32::from(nwave[j]);
                                }
                                v[j] = acc.clamp(-32_768, 32_767) as i16;
                            }
                            v
                        });
                        if let LadderEnv::ImpulseSubslot { .. } = env {
                            // Contaminate one rotating subslot with +15 dB broadband impulse noise
                            let bad_sub = (seed as usize) % REPEATS;
                            let mut st = seed.wrapping_mul(0xD1B5_4A32_D192_ED03) | 1;
                            for sample in &mut subslots[bad_sub] {
                                st ^= st << 13;
                                st ^= st >> 7;
                                st ^= st << 17;
                                let imp = (((st >> 16) as i32 & 0xFFFF) - 32768) / 3;
                                *sample = sample.saturating_add(imp as i16);
                            }
                        }
                        local.push(eval_ladder_minute(&subslots, target, target_hz, 400).as_array());
                    }
                    local
                }));
            }
            for h in handles {
                results.extend(h.join().unwrap());
            }
        });

        for r in &results {
            for m in 0..9 {
                if r[m] {
                    hits[s_idx][m] += 1;
                }
            }
        }
        all_outcomes.push(results);
    }

    // Print per-SNR curve summary
    print!("{:<22}", "stage");
    for &snr in snrs {
        print!(" {:>6.1}", snr);
    }
    println!(" |  SNR50   [95% CI]        dStep");
    let trials_vec = vec![trials_per_snr; snrs.len()];
    let mut prev_50: Option<f64> = None;
    for m in 0..9 {
        let stage_hits: Vec<usize> = (0..snrs.len()).map(|s| hits[s][m]).collect();
        print!("{:<22}", LADDER_NAMES[m]);
        for &h in &stage_hits {
            print!(" {:>5.1}%", 100.0 * h as f64 / trials_per_snr as f64);
        }
        if let Some((x50, _)) = fit_logistic_50(snrs, &stage_hits, &trials_vec) {
            let (lo, hi) = bootstrap_ci_50(snrs, &stage_hits, &trials_vec);
            let delta = prev_50.map_or(String::from("   -- "), |p| format!("{:+6.2}", x50 - p));
            println!(" | {:+6.2} [{:+6.2}, {:+6.2}] {}", x50, lo, hi, delta);
            if m <= 5 {
                prev_50 = Some(x50);
            }
        } else {
            println!(" |    n/a");
        }
    }

    // Print McNemar p-values for key paired transitions at the SNR nearest 50% of hh_prim_oracle
    let hh_hits: Vec<usize> = (0..snrs.len()).map(|s| hits[s][4]).collect();
    if let Some((x50, _)) = fit_logistic_50(snrs, &hh_hits, &trials_vec) {
        let knee_idx = snrs
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| ((*a - x50).abs()).total_cmp(&((*b - x50).abs())))
            .unwrap()
            .0;
        println!("  Paired McNemar at knee SNR = {:+.1} dB:", snrs[knee_idx]);
        for &(a, b, label) in &[
            (0, 1, "lin->log"),
            (1, 2, "log->q6_80ms"),
            (2, 3, "80ms->70ms"),
            (3, 4, "exact->512grid"),
            (4, 5, "bp->bp+osd1"),
            (4, 6, "oracle->prim_blind"),
            (6, 7, "prim_blind->compact"),
            (7, 8, "compact->desktop_dual"),
        ] {
            let mut only_a = 0usize;
            let mut only_b = 0usize;
            for r in &all_outcomes[knee_idx] {
                if r[a] && !r[b] {
                    only_a += 1;
                } else if !r[a] && r[b] {
                    only_b += 1;
                }
            }
            println!(
                "    {:<22}: a_only={:>3}, b_only={:>3}, p={:.4}",
                label,
                only_a,
                only_b,
                mcnemar_p(only_a, only_b)
            );
        }
    }
}

#[derive(Clone, Default)]
struct OracleSummary {
    trials: usize,
    prim_ret: usize,
    comp_ret: usize,
    desk_ret: usize,
    unexpected: usize,
    stage_returned: usize,
    stage_acq: usize,
    stage_cap: usize,
    stage_evid: usize,
    acq_not_shortlisted: usize,
    acq_suppressed: usize,
    acq_compact_rank: usize,
    acq_compact_guard: usize,
    evid_compact_rep: usize,
    evid_compact_nonrep: usize,
}

#[derive(Clone)]
struct SignalSpec {
    frame: Frame,
    hz: u16,
    db: f64,
    offset_ms: u16,
    drift_hz_per_subslot: i16,
    is_target: bool,
}

fn run_oracle_scenario(name: &str, specs: &[SignalSpec], trials: usize, base_seed: u64) {
    let n_threads = thread::available_parallelism().map_or(4, |n| n.get()).min(12);
    let chunk = trials.div_ceil(n_threads);
    let mut total = OracleSummary::default();

    thread::scope(|s| {
        let mut handles = Vec::new();
        for t_idx in 0..n_threads {
            let start = t_idx * chunk;
            let end = (start + chunk).min(trials);
            if start >= end {
                continue;
            }
            handles.push(s.spawn(move || {
                let mut sum = OracleSummary::default();
                let mut rx_c = CompactReceiver60::new();
                let mut rx_dp = Receiver60::new();
                let mut rx_dt = Receiver60::with_percentile_span();
                let target_spec = specs.iter().find(|sp| sp.is_target).unwrap();
                let expected_set: Vec<Frame> = specs.iter().map(|sp| sp.frame).collect();

                for i in start..end {
                    let seed = base_seed.wrapping_add(i as u64);
                    let noise_min = synth_noise_minute(seed);
                    rx_c.reset();
                    rx_dp.reset();
                    rx_dt.reset();

                    for sub in 0..REPEATS {
                        if sub > 0 {
                            rx_c.start_subslot();
                            rx_dp.start_subslot();
                            rx_dt.start_subslot();
                        }
                        let base = sub * RATE as usize * 10;
                        let mut pcm = noise_min[base..base + RATE as usize * 10].to_vec();
                        for sp in specs {
                            let hz = (sp.hz as i32 + (sp.drift_hz_per_subslot as i32) * (sub as i32))
                                .max(300) as u32;
                            let w = synth_subslot_wave(sp.frame, hz, sp.db, sp.offset_ms as usize);
                            for j in 0..pcm.len() {
                                pcm[j] = pcm[j].saturating_add(w[j]);
                            }
                        }
                        for blk in pcm.chunks(BLOCK) {
                            rx_c.push_samples(blk);
                            rx_dp.push_samples(blk);
                            rx_dt.push_samples(blk);
                        }
                    }

                    let probe = [OracleProbe {
                        expected_frame: target_spec.frame,
                        audio_hz: target_spec.hz,
                        offset_ms: target_spec.offset_ms,
                    }];
                    let mut attr = [OracleAttribution::EMPTY];
                    let mut out_c = [Decoded::EMPTY; MAX_DECODED];
                    let (prim_n, comb_n) =
                        rx_c.finish_minute_with_oracle(&mut out_c, &probe, &mut attr);

                    let mut out_dp = [Decoded::EMPTY; MAX_DECODED];
                    let mut out_dt = [Decoded::EMPTY; MAX_DECODED];
                    let n_dp = rx_dp.finish_minute(&mut out_dp);
                    let n_dt = rx_dt.finish_minute(&mut out_dt);

                    sum.trials += 1;
                    if out_c[..prim_n].iter().any(|d| d.frame == target_spec.frame) {
                        sum.prim_ret += 1;
                    }
                    if out_c[..comb_n].iter().any(|d| d.frame == target_spec.frame) {
                        sum.comp_ret += 1;
                    }
                    if out_dp[..n_dp].iter().any(|d| d.frame == target_spec.frame)
                        || out_dt[..n_dt].iter().any(|d| d.frame == target_spec.frame)
                    {
                        sum.desk_ret += 1;
                    }
                    sum.unexpected += out_c[..comb_n]
                        .iter()
                        .filter(|d| !expected_set.contains(&d.frame))
                        .count();

                    let a = attr[0];
                    match a.loss_stage() {
                        LossStage::Returned => sum.stage_returned += 1,
                        LossStage::DroppedOutputCap => sum.stage_cap += 1,
                        LossStage::MissedAcquisition => {
                            sum.stage_acq += 1;
                            if a.primary_oracle_decoded_expected {
                                if a.primary_suppressed {
                                    sum.acq_suppressed += 1;
                                } else if a.primary_shortlist_rank.is_none() {
                                    sum.acq_not_shortlisted += 1;
                                }
                            }
                            if a.compact_oracle_decoded_expected {
                                if a.compact_skipped_primary_guard {
                                    sum.acq_compact_guard += 1;
                                } else if a.compact_sync_rank.is_none() {
                                    sum.acq_compact_rank += 1;
                                }
                            }
                        }
                        LossStage::FailedEvidence => {
                            sum.stage_evid += 1;
                            if a.compact_representable {
                                sum.evid_compact_rep += 1;
                            } else {
                                sum.evid_compact_nonrep += 1;
                            }
                        }
                    }
                }
                sum
            }));
        }
        for h in handles {
            let s = h.join().unwrap();
            total.trials += s.trials;
            total.prim_ret += s.prim_ret;
            total.comp_ret += s.comp_ret;
            total.desk_ret += s.desk_ret;
            total.unexpected += s.unexpected;
            total.stage_returned += s.stage_returned;
            total.stage_acq += s.stage_acq;
            total.stage_cap += s.stage_cap;
            total.stage_evid += s.stage_evid;
            total.acq_not_shortlisted += s.acq_not_shortlisted;
            total.acq_suppressed += s.acq_suppressed;
            total.acq_compact_rank += s.acq_compact_rank;
            total.acq_compact_guard += s.acq_compact_guard;
            total.evid_compact_rep += s.evid_compact_rep;
            total.evid_compact_nonrep += s.evid_compact_nonrep;
        }
    });

    println!(
        "{:<26} | {:>3}/{:<3} {:>3}/{:<3} {:>3}/{:<3} u={:<2} | ret={:<3} acq={:<3} (nsh={:<2} sup={:<2} crk={:<2} cgd={:<2}) cap={:<2} evid={:<3} (rep={:<3}/nr={:<3})",
        name,
        total.prim_ret, total.trials,
        total.comp_ret, total.trials,
        total.desk_ret, total.trials,
        total.unexpected,
        total.stage_returned,
        total.stage_acq,
        total.acq_not_shortlisted,
        total.acq_suppressed,
        total.acq_compact_rank,
        total.acq_compact_guard,
        total.stage_cap,
        total.stage_evid,
        total.evid_compact_rep,
        total.evid_compact_nonrep,
    );
}

fn run_all_oracle(trials: usize) {
    let t = cq(b"NQ6N", 1234);
    let n1 = cq(b"K1ABC", 2345);
    let n2 = cq(b"W1AW", 3456);
    let n3 = cq(b"VE3XYZ", 4567);
    let n4 = cq(b"G4ABC", 5678);

    println!("\n=== ORACLE MISS ATTRIBUTION ({trials} trials/condition, CompactReceiver60) ===");
    println!(
        "{:<26} | {:^26} | attribution breakdown",
        "condition", "prim    comp    desk  unexp"
    );
    println!("{}", "-".repeat(122));

    let scenarios: Vec<(&str, Vec<SignalSpec>)> = vec![
        ("iso_1275_-21dB", vec![SignalSpec { frame: t, hz: 1275, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true }]),
        ("iso_2175_-21dB", vec![SignalSpec { frame: t, hz: 2175, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true }]),
        ("nbr18_1275_440ms", vec![
            SignalSpec { frame: t, hz: 1275, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1050, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_440ms", vec![
            SignalSpec { frame: t, hz: 2175, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_aligned", vec![
            SignalSpec { frame: t, hz: 2175, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_+5Hz", vec![
            SignalSpec { frame: t, hz: 2180, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_-5Hz", vec![
            SignalSpec { frame: t, hz: 2170, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_+10Hz", vec![
            SignalSpec { frame: t, hz: 2185, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_-10Hz", vec![
            SignalSpec { frame: t, hz: 2165, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_+15Hz", vec![
            SignalSpec { frame: t, hz: 2190, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("nbr18_2175_-30Hz", vec![
            SignalSpec { frame: t, hz: 2145, db: -21.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
            SignalSpec { frame: n1, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
        ]),
        ("iso_2175_halfsym_440ms", vec![
            SignalSpec { frame: t, hz: 2175, db: -20.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: true },
        ]),
        ("iso_2175_ramp_+2Hz/sub", vec![
            SignalSpec { frame: t, hz: 2175, db: -20.0, offset_ms: 400, drift_hz_per_subslot: 2, is_target: true },
        ]),
        ("iso_2175_ramp_+5Hz/sub", vec![
            SignalSpec { frame: t, hz: 2175, db: -18.0, offset_ms: 400, drift_hz_per_subslot: 5, is_target: true },
        ]),
        ("crowded_5callers_weak2175", vec![
            SignalSpec { frame: n1, hz: 600, db: -3.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: false },
            SignalSpec { frame: n2, hz: 1050, db: -6.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
            SignalSpec { frame: n3, hz: 1500, db: -10.0, offset_ms: 360, drift_hz_per_subslot: 0, is_target: false },
            SignalSpec { frame: n4, hz: 1950, db: -3.0, offset_ms: 440, drift_hz_per_subslot: 0, is_target: false },
            SignalSpec { frame: t, hz: 2175, db: -20.0, offset_ms: 400, drift_hz_per_subslot: 0, is_target: true },
        ]),
    ];

    for (idx, (name, specs)) in scenarios.iter().enumerate() {
        run_oracle_scenario(name, specs, trials, 50_000 + (idx as u64) * 1_000);
    }
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "all".to_string());
    let arg2: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(100);
    let arg3: usize = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(64);

    if mode == "ladder" || mode == "all" {
        let iso_snrs = [-25.0, -24.5, -24.0, -23.5, -23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0];
        let nbr_snrs = [-24.0, -23.5, -23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];
        let imp_snrs = [-24.0, -23.5, -23.0, -22.5, -22.0, -21.5, -21.0, -20.5, -20.0, -19.5, -19.0];

        run_ladder_condition("isolated_2175_nominal", LadderEnv::Isolated { hz: 2175 }, &iso_snrs, arg2, 100_000);
        run_ladder_condition("isolated_1281_midbin", LadderEnv::Isolated { hz: 1281 }, &iso_snrs, arg2, 200_000);
        run_ladder_condition("neighbor18_2175_440ms", LadderEnv::Neighbor18 { hz: 2175, nbr_hz: 1950 }, &nbr_snrs, arg2, 300_000);
        run_ladder_condition("impulse_subslot_2175", LadderEnv::ImpulseSubslot { hz: 2175 }, &imp_snrs, arg2, 400_000);
    }
    if mode == "oracle" || mode == "all" {
        run_all_oracle(if mode == "oracle" { arg2 } else { arg3 });
    }
}
