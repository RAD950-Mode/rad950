//! Direct HackRF One backend (`2.4 MSPS` half-duplex USB IQ via `hackrf_transfer`).

use std::f64::consts::{PI, TAU};
use std::io::{Read, Write};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use urad950::N_SYMBOLS;

const FS_RF: u64 = 2_400_000;
const IF_OFFSET_HZ: i64 = 240_000;
const DECIM_12K: usize = 200; // 2,400,000 / 12,000 = 200 exact

#[derive(Clone, Debug)]
pub struct HackrfConfig {
    pub serial: Option<String>,
    pub lna_db: u32,
    pub vga_db: u32,
    pub tx_vga_db: u32,
    pub ppm: f64,
    pub dry_tx: bool,
}

impl Default for HackrfConfig {
    fn default() -> Self {
        Self {
            serial: None,
            lna_db: 24,
            vga_db: 20,
            tx_vga_db: 20,
            ppm: 0.0,
            dry_tx: true,
        }
    }
}

pub struct HackrfBackend {
    cfg: HackrfConfig,
    dial_hz: u64,
}

impl HackrfBackend {
    pub fn new(cfg: HackrfConfig, dial_hz: u64) -> Self {
        Self { cfg, dial_hz }
    }

    pub fn set_dial_hz(&mut self, dial_hz: u64) {
        self.dial_hz = dial_hz;
    }

    fn lo_hz(&self) -> u64 {
        let corr = (self.dial_hz as f64) * (1.0 + self.cfg.ppm * 1e-6);
        (corr.round() as i64 - IF_OFFSET_HZ).max(1_000_000) as u64
    }

    pub fn capture_12k(&self, duration_sec: f64, running: &AtomicBool) -> Result<(Vec<i16>, f32), String> {
        let n_iq = ((FS_RF as f64) * duration_sec).round() as u64;
        let mut cmd = Command::new("hackrf_transfer");
        if let Some(ref ser) = self.cfg.serial {
            if !ser.trim().is_empty() {
                cmd.args(["-d", ser.trim()]);
            }
        }
        cmd.args([
            "-r", "-",
            "-f", &self.lo_hz().to_string(),
            "-s", &FS_RF.to_string(),
            "-n", &n_iq.to_string(),
            "-l", &self.cfg.lna_db.to_string(),
            "-g", &self.cfg.vga_db.to_string(),
            "-a", "0",
        ]);
        cmd.stdout(Stdio::piped()).stderr(Stdio::null());
        let mut child = cmd
            .spawn()
            .map_err(|e| format!("failed to launch hackrf_transfer: {e}"))?;
        let mut stdout = child.stdout.take().ok_or("no hackrf stdout")?;

        let chunk_iq = DECIM_12K * 64; // 12,800 IQ samples = 64 output 12 kHz samples
        let mut raw = vec![0u8; chunk_iq * 2];
        let mut pcm_f = Vec::with_capacity((n_iq as usize) / DECIM_12K + 64);

        let dph_rf = -TAU * (IF_OFFSET_HZ as f64 + 1500.0) / (FS_RF as f64);
        let dph_12 = TAU * 1500.0 / 12_000.0;
        let mut ph_rf = 0.0f64;
        let mut ph_12 = 0.0f64;

        while running.load(Ordering::Relaxed) {
            let mut got = 0usize;
            while got < raw.len() {
                match stdout.read(&mut raw[got..]) {
                    Ok(0) => break,
                    Ok(n) => got += n,
                    Err(_) => break,
                }
            }
            if got < DECIM_12K * 2 {
                break;
            }
            let iq_pairs = got / 2;
            let n_out = iq_pairs / DECIM_12K;
            for o in 0..n_out {
                let mut acc_i = 0.0f64;
                let mut acc_q = 0.0f64;
                for k in 0..DECIM_12K {
                    let idx = (o * DECIM_12K + k) * 2;
                    let si = (raw[idx] as i8) as f64 / 128.0;
                    let sq = (raw[idx + 1] as i8) as f64 / 128.0;
                    let (s, c) = ph_rf.sin_cos();
                    acc_i += si * c - sq * s;
                    acc_q += si * s + sq * c;
                    ph_rf += dph_rf;
                    if ph_rf < -TAU {
                        ph_rf += TAU;
                    }
                }
                acc_i /= DECIM_12K as f64;
                acc_q /= DECIM_12K as f64;
                let (s12, c12) = ph_12.sin_cos();
                let usb = (acc_i * c12 - acc_q * s12) as f32;
                ph_12 += dph_12;
                if ph_12 > TAU {
                    ph_12 -= TAU;
                }
                pcm_f.push(usb);
            }
        }
        let _ = child.wait();

        let peak = pcm_f.iter().fold(0.0f32, |m, &x| m.max(x.abs()));
        let scale = if peak > 1e-6 { 22000.0 / peak } else { 1.0 };
        let pcm = pcm_f
            .into_iter()
            .map(|x| (x * scale).clamp(-32767.0, 32767.0) as i16)
            .collect();
        Ok((pcm, (peak * 100.0).min(100.0)))
    }

    pub fn transmit_burst(
        &self,
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

        let mut cmd = Command::new("hackrf_transfer");
        if let Some(ref ser) = self.cfg.serial {
            if !ser.trim().is_empty() {
                cmd.args(["-d", ser.trim()]);
            }
        }
        cmd.args([
            "-t", "-",
            "-f", &self.lo_hz().to_string(),
            "-s", &FS_RF.to_string(),
            "-x", &self.cfg.tx_vga_db.to_string(),
            "-a", "0",
        ]);
        cmd.stdin(Stdio::piped()).stdout(Stdio::null()).stderr(Stdio::null());
        let mut child = cmd
            .spawn()
            .map_err(|e| format!("failed to launch hackrf_transfer TX: {e}"))?;
        let mut stdin = child.stdin.take().ok_or("no hackrf stdin")?;

        let nsps = (FS_RF as f64 * 0.080).round() as usize;
        let nramp = (FS_RF as f64 * 0.010).round() as usize;
        let mut phase = 0.0f64;
        let base_off = IF_OFFSET_HZ as f64 + audio_hz as f64;
        let mut f_prev = base_off + (tones[0] as f64) * 50.0;
        let mut buf = vec![0u8; nsps * 2];

        for (isym, &tone) in tones.iter().enumerate() {
            if !running.load(Ordering::Relaxed) {
                break;
            }
            let f_curr = base_off + (tone as f64) * 50.0;
            for i in 0..nsps {
                let f_inst = if isym > 0 && i < nramp {
                    let alpha = 0.5 * (1.0 - (PI * i as f64 / nramp as f64).cos());
                    f_prev + alpha * (f_curr - f_prev)
                } else {
                    f_curr
                };
                phase = (phase + TAU * f_inst / (FS_RF as f64)) % TAU;
                let (s, c) = phase.sin_cos();
                buf[i * 2] = (c * 100.0).round() as i8 as u8;
                buf[i * 2 + 1] = (s * 100.0).round() as i8 as u8;
            }
            f_prev = f_curr;
            if stdin.write_all(&buf).is_err() {
                break;
            }
        }
        drop(stdin);
        let _ = child.wait();
        Ok(())
    }
}
