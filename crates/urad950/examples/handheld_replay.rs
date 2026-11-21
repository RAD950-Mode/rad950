//! Deterministic, host-only replay for the compact RAD950-60 receiver.
//!
//! It exercises the public streaming API that a handheld firmware uses.  The
//! `primary` count is the set produced by the compact receiver's primary fold;
//! `combined` includes frames found by its bounded alternate pass.

use urad950::{
    encode_tones, Callsign, CompactReceiver60, Decoded, Frame, Modulator, DEFAULT_SPACING_HZ,
    MAX_DECODED,
};

const RATE: u32 = 24_000;
const SIGMA: f64 = 1500.0;
const REPEATS: usize = 5;
const BLOCK: usize = 192;

fn cq(call: &[u8]) -> Frame {
    Frame::Cq {
        sender: Callsign::encode(call).unwrap(),
        grid4: 1234,
        tx_pwr: 0,
        reserved: 0,
    }
}

fn waveform(frame: Frame, hz: u32, db: f64, offset_ms: usize) -> Vec<i16> {
    let noise_power = SIGMA.powi(2) * 2500.0 / (RATE / 2) as f64;
    let amplitude = (2.0 * noise_power * 10f64.powf(db / 10.0)).sqrt().round() as i16;
    let mut wave = vec![0; RATE as usize * 10];
    let mut tx = Modulator::new(encode_tones(frame), RATE, hz, DEFAULT_SPACING_HZ, amplitude);
    for sample in &mut wave[offset_ms * 24..] {
        let Some(value) = tx.next_sample() else { break };
        *sample = value;
    }
    wave
}

fn noise(seed: u64) -> Vec<i16> {
    let mut state = seed;
    (0..RATE as usize * 10 * REPEATS)
        .map(|_| {
            let mut sum = -6.0;
            for _ in 0..12 {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                sum += (state >> 16) as u32 as f64 / u32::MAX as f64;
            }
            (sum * SIGMA).round() as i16
        })
        .collect()
}

fn decode(noise: &[i16], target: &[i16], neighbor: &[i16]) -> (Vec<Frame>, usize) {
    let mut rx = CompactReceiver60::new();
    for (subslot, noise_part) in noise.chunks_exact(target.len()).enumerate() {
        if subslot > 0 {
            rx.start_subslot();
        }
        for (block, noise_part) in noise_part.chunks_exact(BLOCK).enumerate() {
            let samples: [i16; BLOCK] = core::array::from_fn(|i| {
                noise_part[i]
                    .saturating_add(target[block * BLOCK + i])
                    .saturating_add(neighbor[block * BLOCK + i])
            });
            rx.push_samples(&samples);
        }
    }
    let mut out = [Decoded::EMPTY; MAX_DECODED];
    let (primary, combined) = rx.finish_minute_with_counts(&mut out);
    assert!(primary <= combined && combined <= MAX_DECODED);
    (
        out[..combined]
            .iter()
            .map(|decoded| decoded.frame)
            .collect(),
        primary,
    )
}

fn decode_noise_only(seed: u64) -> usize {
    let random = noise(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    let silent = vec![0; RATE as usize * 10];
    decode(&random, &silent, &silent).0.len()
}

fn run_trial(seed: u64, hz: u32) -> (bool, bool, bool, usize) {
    let target = cq(b"NQ6N");
    let neighbor = cq(b"K1ABC");
    let random = noise(seed.wrapping_mul(0x9e37_79b9_7f4a_7c15));
    let weak = waveform(target, hz, -21.0, 400);
    let strong = waveform(neighbor, hz - 225, -3.0, 440);
    let (frames, primary) = decode(&random, &weak, &strong);
    (
        frames[..primary].contains(&target),
        frames.contains(&target),
        frames.contains(&neighbor),
        frames
            .iter()
            .filter(|&&frame| frame != target && frame != neighbor)
            .count(),
    )
}

fn main() {
    let trials: usize = std::env::args()
        .nth(1)
        .map_or(48, |value| value.parse().unwrap());
    let first_seed: u64 = std::env::args()
        .nth(2)
        .map_or(12_001, |value| value.parse().unwrap());
    assert!(trials > 0);
    println!("frequency_hz,trials,target_primary,target_combined,neighbor_combined,unexpected");
    for hz in [1275, 2175] {
        let mut primary = 0;
        let mut combined = 0;
        let mut neighbor = 0;
        let mut unexpected = 0;
        for seed in first_seed..first_seed + trials as u64 {
            let (got_primary, got_combined, got_neighbor, false_frames) = run_trial(seed, hz);
            primary += got_primary as usize;
            combined += got_combined as usize;
            neighbor += got_neighbor as usize;
            unexpected += false_frames;
        }
        println!("{hz},{trials},{primary},{combined},{neighbor},{unexpected}");
    }
    let noise_false = (0..trials)
        .map(|offset| decode_noise_only(first_seed + offset as u64))
        .sum::<usize>();
    println!("noise_only,{trials},0,0,0,{noise_false}");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn upper_channel_replay_uses_compact_pass_without_extra_frames() {
        let (primary, combined, _neighbor, unexpected) = run_trial(12_003, 2175);
        assert!(
            !primary,
            "test seed must require the compact alternate pass"
        );
        assert!(
            combined,
            "compact alternate pass did not recover the upper-channel target"
        );
        assert_eq!(unexpected, 0);
    }

    #[test]
    fn compact_receiver_rejects_deterministic_noise_only_minutes() {
        for seed in 12_001..12_005 {
            assert_eq!(
                decode_noise_only(seed),
                0,
                "noise-only seed {seed} produced a frame"
            );
        }
    }
}
