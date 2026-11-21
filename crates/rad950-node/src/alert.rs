//! Optional non-blocking audio alert chimes on the system default output device.
//!
//! Matches the RT-950 Pro's acoustic signature:
//! - [`AlertTone::NewHeard`]: single crisp G6 (`1568 Hz`, 180 ms) ding when a new station/grid CQ is heard.
//! - [`AlertTone::NewWorked`]: rising fifth C6 (`1047 Hz`, 110 ms) → G6 (`1568 Hz`, 230 ms) when a new one is worked.

use std::f32::consts::TAU;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::SampleFormat;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlertTone {
    NewHeard,
    NewWorked,
}

/// Synthesize a click-free sequence of `(freq_hz, duration_ms)` notes at `sample_rate`.
pub fn synth_chime_f32(notes: &[(f32, u32)], sample_rate: u32, gain: f32) -> Vec<f32> {
    let fade_len = ((sample_rate as f32) * 0.008).max(1.0) as usize; // 8 ms raised-cosine fade
    let mut out = Vec::new();
    for &(freq, ms) in notes {
        let n = ((sample_rate as u64 * ms as u64) / 1000).max(1) as usize;
        for i in 0..n {
            let env = if i < fade_len {
                0.5 * (1.0 - (std::f32::consts::PI * (i as f32) / (fade_len as f32)).cos())
            } else if i + fade_len >= n {
                let rem = n - 1 - i;
                0.5 * (1.0 - (std::f32::consts::PI * (rem as f32) / (fade_len as f32)).cos())
            } else {
                1.0
            };
            let s = if freq > 0.0 {
                (TAU * freq * (i as f32) / (sample_rate as f32)).sin() * gain * env
            } else {
                0.0
            };
            out.push(s);
        }
    }
    // 25 ms trailing silence so the OS audio buffer drains cleanly without popping
    out.extend(std::iter::repeat_n(0.0f32, (sample_rate / 40) as usize));
    out
}

/// Play an alert chime asynchronously on the system's default output speaker.
/// Fails silently if no audio output device is available.
pub fn play_alert_tone(kind: AlertTone) {
    thread::spawn(move || {
        let _ = play_blocking(kind);
    });
}

fn play_blocking(kind: AlertTone) -> Result<(), String> {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .ok_or_else(|| "no default output device".to_string())?;
    let cfg = device
        .default_output_config()
        .map_err(|e| format!("output config: {e}"))?;
    let sr = cfg.sample_rate().0;
    let channels = cfg.channels() as usize;

    let notes: &[(f32, u32)] = match kind {
        AlertTone::NewHeard => &[(1568.0, 180)],
        AlertTone::NewWorked => &[(1047.0, 110), (1568.0, 230)],
    };
    let mono = synth_chime_f32(notes, sr, 0.20);
    let total_ms = (mono.len() as u64 * 1000) / (sr as u64).max(1) + 60;
    let samples = Arc::new(mono);
    let pos = Arc::new(Mutex::new(0usize));

    let err_fn = |_err| {};
    let stream = match cfg.sample_format() {
        SampleFormat::F32 => {
            let s = Arc::clone(&samples);
            let p = Arc::clone(&pos);
            device.build_output_stream(
                &cfg.into(),
                move |data: &mut [f32], _| {
                    let mut idx = p.lock().unwrap();
                    for frame in data.chunks_mut(channels) {
                        let v = s.get(*idx).copied().unwrap_or(0.0);
                        *idx = idx.saturating_add(1);
                        for ch in frame.iter_mut() {
                            *ch = v;
                        }
                    }
                },
                err_fn,
                None,
            )
        }
        SampleFormat::I16 => {
            let s = Arc::clone(&samples);
            let p = Arc::clone(&pos);
            device.build_output_stream(
                &cfg.into(),
                move |data: &mut [i16], _| {
                    let mut idx = p.lock().unwrap();
                    for frame in data.chunks_mut(channels) {
                        let v = (s.get(*idx).copied().unwrap_or(0.0) * 32767.0)
                            .clamp(-32767.0, 32767.0) as i16;
                        *idx = idx.saturating_add(1);
                        for ch in frame.iter_mut() {
                            *ch = v;
                        }
                    }
                },
                err_fn,
                None,
            )
        }
        SampleFormat::U16 => {
            let s = Arc::clone(&samples);
            let p = Arc::clone(&pos);
            device.build_output_stream(
                &cfg.into(),
                move |data: &mut [u16], _| {
                    let mut idx = p.lock().unwrap();
                    for frame in data.chunks_mut(channels) {
                        let v = ((s.get(*idx).copied().unwrap_or(0.0) * 0.5 + 0.5) * 65535.0)
                            .clamp(0.0, 65535.0) as u16;
                        *idx = idx.saturating_add(1);
                        for ch in frame.iter_mut() {
                            *ch = v;
                        }
                    }
                },
                err_fn,
                None,
            )
        }
        other => return Err(format!("unsupported sample format: {other:?}")),
    }
    .map_err(|e| format!("build_output_stream: {e}"))?;

    stream.play().map_err(|e| format!("play: {e}"))?;
    thread::sleep(Duration::from_millis(total_ms));
    drop(stream);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synth_chime_lengths_and_bounds() {
        let heard = synth_chime_f32(&[(1568.0, 180)], 48_000, 0.20);
        let worked = synth_chime_f32(&[(1047.0, 110), (1568.0, 230)], 48_000, 0.20);
        assert!(worked.len() > heard.len());
        assert!(heard.iter().all(|&s| s.abs() <= 0.21));
        assert!(worked.iter().all(|&s| s.abs() <= 0.21));
    }
}
