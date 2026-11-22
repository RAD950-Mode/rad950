//! Raised-cosine-shaped audio CW ID synthesizer (`DE <CALL> RAD950.ORG`).

use std::f64::consts::TAU;

fn morse_of(ch: char) -> Option<&'static str> {
    match ch.to_ascii_uppercase() {
        'A' => Some(".-"),
        'B' => Some("-..."),
        'C' => Some("-.-."),
        'D' => Some("-.."),
        'E' => Some("."),
        'F' => Some("..-."),
        'G' => Some("--."),
        'H' => Some("...."),
        'I' => Some(".."),
        'J' => Some(".---"),
        'K' => Some("-.-"),
        'L' => Some(".-.."),
        'M' => Some("--"),
        'N' => Some("-."),
        'O' => Some("---"),
        'P' => Some(".--."),
        'Q' => Some("--.-"),
        'R' => Some(".-."),
        'S' => Some("..."),
        'T' => Some("-"),
        'U' => Some("..-"),
        'V' => Some("...-"),
        'W' => Some(".--"),
        'X' => Some("-..-"),
        'Y' => Some("-.--"),
        'Z' => Some("--.."),
        '0' => Some("-----"),
        '1' => Some(".----"),
        '2' => Some("..---"),
        '3' => Some("...--"),
        '4' => Some("....-"),
        '5' => Some("....."),
        '6' => Some("-...."),
        '7' => Some("--..."),
        '8' => Some("---.."),
        '9' => Some("----."),
        '/' => Some("-..-."),
        '.' => Some(".-.-.-"),
        '-' => Some("-....-"),
        _ => None,
    }
}

/// Synthesize a clean, raised-cosine-shaped CW tone waveform at `audio_hz` and `sample_rate`.
/// Uses 5 ms rise/fall ramps to eliminate key clicks.
pub fn synth_cw_f32(
    text: &str,
    audio_hz: f64,
    sample_rate: u32,
    wpm: f32,
    gain: f32,
) -> Vec<f32> {
    let wpm = wpm.clamp(12.0, 35.0) as f64;
    let gain = gain.clamp(0.05, 0.95) as f64;

    // Build keying segments as (key_down, duration_in_dits)
    let mut segs: Vec<(bool, usize)> = Vec::new();
    let mut prev_was_char = false;
    for ch in text.chars() {
        if ch.is_whitespace() {
            if prev_was_char {
                // Word gap = 7 dits total (already have 0 after last element; char gap not yet added)
                segs.push((false, 7));
                prev_was_char = false;
            }
            continue;
        }
        let Some(code) = morse_of(ch) else { continue };
        if prev_was_char {
            segs.push((false, 3)); // inter-character gap
        }
        for (i, el) in code.bytes().enumerate() {
            if i > 0 {
                segs.push((false, 1)); // intra-character gap
            }
            segs.push((true, if el == b'-' { 3 } else { 1 }));
        }
        prev_was_char = true;
    }

    let total_dits: usize = segs.iter().map(|(_, d)| *d).sum();
    // Auto-scale WPM upward if a long callsign would exceed 8.5 s so it always fits inside a 10 s slot.
    let min_wpm_for_slot = (total_dits as f64 * 1.2) / 8.5;
    let eff_wpm = wpm.max(min_wpm_for_slot).clamp(12.0, 35.0);
    let dit_samples = ((1.2 / eff_wpm) * sample_rate as f64).round().max(1.0) as usize;
    let ramp = ((0.005 * sample_rate as f64).round() as usize)
        .min(dit_samples / 2)
        .max(1);
    let mut out = Vec::with_capacity(total_dits * dit_samples + sample_rate as usize / 10);
    let mut phase = 0.0f64;
    let dphase = TAU * audio_hz / sample_rate as f64;

    for (key_down, dits) in segs {
        let n = dits * dit_samples;
        if !key_down {
            out.resize(out.len() + n, 0.0);
            phase = (phase + dphase * n as f64) % TAU;
        } else {
            for i in 0..n {
                let env = if i < ramp {
                    0.5 * (1.0 - (std::f64::consts::PI * i as f64 / ramp as f64).cos())
                } else if i >= n - ramp {
                    let k = n - 1 - i;
                    0.5 * (1.0 - (std::f64::consts::PI * k as f64 / ramp as f64).cos())
                } else {
                    1.0
                };
                out.push((phase.sin() * gain * env) as f32);
                phase += dphase;
                if phase >= TAU {
                    phase -= TAU;
                }
            }
        }
    }

    out
}

/// Format the standard RAD-950 CW ID message (stripping any `-N` numeric suffix from callsign).
pub fn format_cw_id_text(call_disp: &str) -> String {
    let base = call_disp.split('-').next().unwrap_or(call_disp).trim();
    format!("{base} RAD950.ORG")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cw_id_fits_inside_10s_slot() {
        for (call, want) in [("NQ6N-2", "NQ6N RAD950.ORG"), ("WA1ABC", "WA1ABC RAD950.ORG")] {
            let msg = format_cw_id_text(call);
            assert_eq!(msg, want);
            let pcm = synth_cw_f32(&msg, 1500.0, 48_000, 25.0, 0.5);
            let dur_sec = pcm.len() as f64 / 48_000.0;
            assert!(dur_sec > 5.0 && dur_sec <= 8.6, "{msg} duration {dur_sec:.2}s should fit in 10s slot");
            let peak = pcm.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
            assert!(peak > 0.45 && peak <= 0.501, "peak={peak}");
        }
    }
}
