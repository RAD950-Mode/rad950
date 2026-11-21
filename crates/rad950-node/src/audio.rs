//! Cross-platform audio backend (`cpal`: macOS CoreAudio, Linux ALSA/PipeWire, Windows WASAPI)
//! paired with Hamlib CAT/PTT (`rigctld`).

use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use urad950::N_SYMBOLS;

use crate::hamlib::{HamlibClient, HamlibConfig};

pub const RX_RATE: u32 = 12_000;

#[derive(Clone, Debug)]
pub struct AudioConfig {
    pub input_device: Option<String>,
    pub output_device: Option<String>,
    pub tx_gain: f32, // 0.0..=1.0
    pub dry_tx: bool,
    pub hamlib: HamlibConfig,
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            input_device: None,
            output_device: None,
            tx_gain: 0.5,
            dry_tx: true,
            hamlib: HamlibConfig::default(),
        }
    }
}

pub fn list_audio_devices() -> (Vec<String>, Vec<String>) {
    let host = cpal::default_host();
    let ins = host
        .input_devices()
        .map(|it| it.filter_map(|d| d.name().ok()).collect())
        .unwrap_or_default();
    let outs = host
        .output_devices()
        .map(|it| it.filter_map(|d| d.name().ok()).collect())
        .unwrap_or_default();
    (ins, outs)
}

/// Synthesize RAD950 spec v1 continuous-phase raised-cosine 4-FSK mono `f32` audio.
/// `110` symbols × `80 ms`, `50 Hz` spacing, `10 ms` raised-cosine frequency ramps & burst edges.
pub fn synth_4fsk_f32(tones: &[u8; N_SYMBOLS], audio_hz: f64, sample_rate: u32, gain: f32) -> Vec<f32> {
    let fs = sample_rate as f64;
    let nsps = (fs * 0.080).round() as usize;
    let nramp = ((fs * 0.010).round() as usize).max(1);
    let total = N_SYMBOLS * nsps;
    let mut out = Vec::with_capacity(total);

    let mut phase = 0.0f64;
    let mut f_prev = audio_hz + (tones[0] as f64) * 50.0;
    let mut idx = 0usize;

    for (isym, &tone) in tones.iter().enumerate() {
        let f_curr = audio_hz + (tone as f64) * 50.0;
        for i in 0..nsps {
            let f_inst = if isym > 0 && i < nramp {
                let alpha = 0.5 * (1.0 - (PI * i as f64 / nramp as f64).cos());
                f_prev + alpha * (f_curr - f_prev)
            } else {
                f_curr
            };
            let env = if idx < nramp {
                0.5 * (1.0 - (PI * idx as f64 / nramp as f64).cos())
            } else if idx >= total - nramp {
                0.5 * (1.0 + (PI * (idx - (total - nramp)) as f64 / nramp as f64).cos())
            } else {
                1.0
            };
            phase = (phase + TAU * f_inst / fs) % TAU;
            out.push((env * phase.sin()) as f32 * gain);
            idx += 1;
        }
        f_prev = f_curr;
    }
    out
}

pub struct AudioBackend {
    cfg: AudioConfig,
    hamlib: HamlibClient,
}

impl AudioBackend {
    pub fn new(cfg: AudioConfig) -> Self {
        let hamlib = HamlibClient::new(cfg.hamlib.clone());
        Self { cfg, hamlib }
    }

    pub fn start_cat(&mut self, dial_hz: u64) -> Result<(), String> {
        if self.hamlib.is_configured() {
            self.hamlib.start_if_needed()?;
            self.hamlib.set_mode_usb()?;
            self.hamlib.set_freq(dial_hz)?;
        }
        Ok(())
    }

    pub fn set_dial_hz(&mut self, dial_hz: u64) -> Result<(), String> {
        self.hamlib.set_freq(dial_hz)
    }

    fn find_input_device(&self) -> Result<cpal::Device, String> {
        let host = cpal::default_host();
        if let Some(ref want) = self.cfg.input_device {
            if !want.trim().is_empty() {
                if let Ok(mut devs) = host.input_devices() {
                    if let Some(d) = devs.find(|d| d.name().ok().is_some_and(|n| n.contains(want))) {
                        return Ok(d);
                    }
                }
            }
        }
        host.default_input_device()
            .ok_or_else(|| "no default audio input device found".to_string())
    }

    fn find_output_device(&self) -> Result<cpal::Device, String> {
        let host = cpal::default_host();
        if let Some(ref want) = self.cfg.output_device {
            if !want.trim().is_empty() {
                if let Ok(mut devs) = host.output_devices() {
                    if let Some(d) = devs.find(|d| d.name().ok().is_some_and(|n| n.contains(want))) {
                        return Ok(d);
                    }
                }
            }
        }
        host.default_output_device()
            .ok_or_else(|| "no default audio output device found".to_string())
    }

    /// Capture `duration_sec` of mono audio and resample to 12,000 Hz `i16`.
    pub fn capture_12k(
        &mut self,
        duration_sec: f64,
        running: &AtomicBool,
    ) -> Result<(Vec<i16>, f32), String> {
        let dev = self.find_input_device()?;
        let def_cfg = dev
            .default_input_config()
            .map_err(|e| format!("audio input config error: {e}"))?;
        let host_rate = def_cfg.sample_rate().0;
        let channels = def_cfg.channels() as usize;
        let need_host = ((host_rate as f64) * duration_sec).round() as usize;
        let buf = Arc::new(Mutex::new(Vec::<f32>::with_capacity(need_host + 4096)));
        let buf_cb = Arc::clone(&buf);

        let stream = match def_cfg.sample_format() {
            cpal::SampleFormat::F32 => dev.build_input_stream(
                &def_cfg.into(),
                move |data: &[f32], _| {
                    let mut b = buf_cb.lock().unwrap();
                    for frame in data.chunks_exact(channels) {
                        b.push(frame[0]);
                    }
                },
                |_| {},
                None,
            ),
            cpal::SampleFormat::I16 => dev.build_input_stream(
                &def_cfg.into(),
                move |data: &[i16], _| {
                    let mut b = buf_cb.lock().unwrap();
                    for frame in data.chunks_exact(channels) {
                        b.push(frame[0] as f32 / 32768.0);
                    }
                },
                |_| {},
                None,
            ),
            cpal::SampleFormat::U16 => dev.build_input_stream(
                &def_cfg.into(),
                move |data: &[u16], _| {
                    let mut b = buf_cb.lock().unwrap();
                    for frame in data.chunks_exact(channels) {
                        b.push((frame[0] as f32 - 32768.0) / 32768.0);
                    }
                },
                |_| {},
                None,
            ),
            other => return Err(format!("unsupported audio input sample format {other:?}")),
        }
        .map_err(|e| format!("failed to open audio input stream: {e}"))?;

        stream.play().map_err(|e| format!("audio input play failed: {e}"))?;

        let deadline = Instant::now() + Duration::from_secs_f64(duration_sec + 1.0);
        while running.load(Ordering::Relaxed) && Instant::now() < deadline {
            if buf.lock().unwrap().len() >= need_host {
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        drop(stream);

        let samples = buf.lock().unwrap();
        let mut peak = 0.0f32;
        for &x in samples.iter() {
            let a = x.abs();
            if a > peak {
                peak = a;
            }
        }
        let pcm = resample_to_12k(&samples, host_rate);
        Ok((pcm, (peak * 100.0).min(100.0)))
    }

    /// Transmit one RAD950 8.80 s burst (with Hamlib PTT if not `dry_tx`).
    pub fn transmit_burst(
        &mut self,
        tones: &[u8; N_SYMBOLS],
        audio_hz: u16,
        running: &AtomicBool,
    ) -> Result<(), String> {
        if self.cfg.dry_tx {
            let deadline = Instant::now() + Duration::from_millis(8800);
            while running.load(Ordering::Relaxed) && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(50));
            }
            return Ok(());
        }

        let dev = self.find_output_device()?;
        let def_cfg = dev
            .default_output_config()
            .map_err(|e| format!("audio output config error: {e}"))?;
        let host_rate = def_cfg.sample_rate().0;
        let channels = def_cfg.channels() as usize;

        let wave = synth_4fsk_f32(tones, audio_hz as f64, host_rate, self.cfg.tx_gain.clamp(0.05, 0.95));
        let queue = Arc::new(Mutex::new(VecDeque::from(wave)));
        let q_cb = Arc::clone(&queue);
        let done = Arc::new(AtomicBool::new(false));
        let done_cb = Arc::clone(&done);

        let stream = match def_cfg.sample_format() {
            cpal::SampleFormat::F32 => dev.build_output_stream(
                &def_cfg.into(),
                move |data: &mut [f32], _| {
                    let mut q = q_cb.lock().unwrap();
                    for frame in data.chunks_exact_mut(channels) {
                        let s = q.pop_front().unwrap_or(0.0);
                        for ch in frame.iter_mut() {
                            *ch = s;
                        }
                    }
                    if q.is_empty() {
                        done_cb.store(true, Ordering::Relaxed);
                    }
                },
                |_| {},
                None,
            ),
            cpal::SampleFormat::I16 => dev.build_output_stream(
                &def_cfg.into(),
                move |data: &mut [i16], _| {
                    let mut q = q_cb.lock().unwrap();
                    for frame in data.chunks_exact_mut(channels) {
                        let s = (q.pop_front().unwrap_or(0.0) * 32767.0).clamp(-32767.0, 32767.0) as i16;
                        for ch in frame.iter_mut() {
                            *ch = s;
                        }
                    }
                    if q.is_empty() {
                        done_cb.store(true, Ordering::Relaxed);
                    }
                },
                |_| {},
                None,
            ),
            other => return Err(format!("unsupported audio output sample format {other:?}")),
        }
        .map_err(|e| format!("failed to open audio output stream: {e}"))?;

        let _ = self.hamlib.set_ptt(true);
        let play_res = stream.play().map_err(|e| format!("audio output play failed: {e}"));
        if play_res.is_ok() {
            let deadline = Instant::now() + Duration::from_secs(10);
            while running.load(Ordering::Relaxed)
                && !done.load(Ordering::Relaxed)
                && Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(20));
            }
            std::thread::sleep(Duration::from_millis(60));
        }
        drop(stream);
        let _ = self.hamlib.set_ptt(false);
        play_res
    }
}

fn resample_to_12k(input: &[f32], host_rate: u32) -> Vec<i16> {
    if input.is_empty() || host_rate == 0 {
        return Vec::new();
    }
    if host_rate == RX_RATE {
        return input
            .iter()
            .map(|&x| (x * 32767.0).clamp(-32767.0, 32767.0) as i16)
            .collect();
    }
    if host_rate % RX_RATE == 0 {
        let decim = (host_rate / RX_RATE) as usize;
        return input
            .chunks_exact(decim)
            .map(|c| {
                let avg: f32 = c.iter().sum::<f32>() / (decim as f32);
                (avg * 32767.0).clamp(-32767.0, 32767.0) as i16
            })
            .collect();
    }
    // General linear resampler (e.g. 44,100 Hz -> 12,000 Hz)
    let ratio = host_rate as f64 / RX_RATE as f64;
    let n_out = ((input.len() as f64) / ratio) as usize;
    let mut out = Vec::with_capacity(n_out);
    for i in 0..n_out {
        let pos = i as f64 * ratio;
        let idx = pos as usize;
        let frac = (pos - idx as f64) as f32;
        let a = input[idx];
        let b = input.get(idx + 1).copied().unwrap_or(a);
        let v = a + frac * (b - a);
        out.push((v * 32767.0).clamp(-32767.0, 32767.0) as i16);
    }
    out
}
