//! Serial-connected Radtel RT-950 Pro (Ferrite Link v1) backend for `rad950-node`.
//!
//! Speaks Ferrite's COBS + CRC-16/CCITT-FALSE link protocol over the CH340 USB programming cable:
//! - Automatically enters `RAD950` mode on the radio (`MENU 0 3 7`) if not already open.
//! - Upgrades to `230,400` baud and streams live `12,000 Hz` G.711 µ-law RX audio (`AUDIO` / `AUDIO_DATA`)
//!   into `rad950-node`'s `urad950` decoder and `386`-byte desktop/web panafall streamer.
//! - Switches HF bands (`17m / 15m / 12m / 10m`) via remote `KNOB` steps on `rad950.operate`.
//! - Exposes the serial-connected RT-950 Pro directly on `https://rad950.org/on-the-air/remote`
//!   using the exact same Web Remote Control UI as desktop/HackRF stations.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use serialport::SerialPort;

const FRAME_HELLO: u8 = 0x01;
const FRAME_BAUD: u8 = 0x20;
const FRAME_KEY: u8 = 0x30;
const FRAME_UI: u8 = 0x33;
const FRAME_AUDIO: u8 = 0x56;
const FRAME_AUDIO_DATA: u8 = 0x57;
const FRAME_CLOCK: u8 = 0x63;
const FRAME_RAD950_TX: u8 = 0x65;

/// Link v1 HELLO payload: protocol=1, lz=0 (uncompressed), 7 reserved bytes, client name.
const HELLO_PAYLOAD: &[u8] = &[1, 0, 0, 0, 0, 0, 0, 0, 0, b'r', b'a', b'd', b'9', b'5', b'0'];

const KEY_MENU: u8 = 6;
const KEY_BACK: u8 = 9;
const KEY_0: u8 = 17;
const KEY_3: u8 = 12;
const KEY_7: u8 = 18;
const KEY_SHORT: u8 = 2;
const KEY_KNOB: u8 = 0x81;

#[derive(Clone, Debug)]
pub struct Rt950Config {
    pub port: String,
    pub dry_tx: bool,
}

pub struct Rt950Backend {
    cfg: Rt950Config,
    port: Option<Box<dyn SerialPort>>,
    seq: u8,
    rx_buf: Vec<u8>,
    dial_hz: u64,
    band_slot: i32,
    audio_on: bool,
    fast_baud: bool,
    last_audio_renew: Instant,
}

impl Rt950Backend {
    pub fn new(cfg: Rt950Config, dial_hz: u64) -> Self {
        Self {
            cfg,
            port: None,
            seq: 1,
            rx_buf: Vec::with_capacity(4096),
            dial_hz,
            band_slot: dial_to_band_slot(dial_hz),
            audio_on: false,
            fast_baud: false,
            last_audio_renew: Instant::now(),
        }
    }

    pub fn set_dry_tx(&mut self, dry_tx: bool) {
        self.cfg.dry_tx = dry_tx;
    }

    /// Open the serial cable, acquire Ferrite's link, ensure RAD950 mode is open, and upgrade to 230,400 baud.
    pub fn connect(&mut self) -> Result<(), String> {
        let port_name = self.cfg.port.trim();
        if port_name.is_empty() {
            return Err("rt950_port is empty (pass --rt950-port /dev/ttyUSB0)".to_string());
        }
        let mut ser = serialport::new(port_name, 115_200)
            .timeout(Duration::from_millis(40))
            .open()
            .map_err(|e| format!("open {port_name}: {e}"))?;

        // Take the cable from Ferrite's terminal (TERM-2: Ctrl-C then 0x00)
        let _ = ser.write_all(&[0x03]);
        let _ = ser.flush();
        thread::sleep(Duration::from_millis(250));
        let mut drain = [0u8; 512];
        let _ = ser.read(&mut drain);
        let _ = ser.write_all(&[0x00]);
        let _ = ser.flush();

        self.port = Some(ser);
        self.rx_buf.clear();
        self.fast_baud = false;
        self.audio_on = false;

        // Verify HELLO at 115,200 baud (or recover if radio was left at 230,400 baud)
        if self.rpc(FRAME_HELLO, HELLO_PAYLOAD).is_err() {
            if let Some(p) = self.port.as_mut() {
                let _ = p.set_baud_rate(230_400);
            }
            self.rx_buf.clear();
            self.rpc(FRAME_HELLO, HELLO_PAYLOAD)?;
            self.fast_baud = true;
        }

        // Sync radio's UTC clock from host NTP so on-radio display & slot clock match
        if let Ok(dur) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
            let utc_sec = dur.as_secs() as u32;
            let _ = self.rpc(FRAME_CLOCK, &utc_sec.to_le_bytes());
        }

        // Check compact status (`UI what=2`): if not already in `mode=rad950`, navigate `MENU 0 3 7`
        if let Ok(st_bytes) = self.rpc(FRAME_UI, &[2]) {
            let st = String::from_utf8_lossy(&st_bytes);
            if !st.contains("mode=rad950") {
                // Return to Home safely, then open Tools -> RAD950 (`MENU 0`, `3`, `7`)
                for _ in 0..4 {
                    let _ = self.send_key(KEY_BACK);
                    thread::sleep(Duration::from_millis(120));
                }
                let _ = self.send_key(KEY_MENU);
                thread::sleep(Duration::from_millis(180));
                let _ = self.send_key(KEY_0);
                thread::sleep(Duration::from_millis(250));
                let _ = self.send_key(KEY_3);
                thread::sleep(Duration::from_millis(250));
                let _ = self.send_key(KEY_7);
                thread::sleep(Duration::from_millis(1200));
            }
        }

        // Upgrade cable to 230,400 baud for lossless 12 kHz µ-law RX audio streaming (LINK-11)
        if !self.fast_baud {
            self.rpc(FRAME_BAUD, &230_400u32.to_le_bytes())
                .map_err(|e| format!("BAUD 230400 request failed: {e}"))?;
            thread::sleep(Duration::from_millis(60));
            if let Some(p) = self.port.as_mut() {
                p.set_baud_rate(230_400)
                    .map_err(|e| format!("host set_baud_rate(230400) failed: {e}"))?;
            }
            self.rx_buf.clear();
            self.rpc(FRAME_HELLO, HELLO_PAYLOAD)
                .map_err(|e| format!("HELLO at 230400 failed: {e}"))?;
            self.fast_baud = true;
        }

        self.start_audio()?;
        Ok(())
    }

    pub fn disconnect(&mut self) {
        if self.port.is_none() {
            return;
        }
        if self.audio_on {
            let _ = self.rpc(FRAME_AUDIO, &[0]);
            self.audio_on = false;
        }
        if self.fast_baud {
            let _ = self.rpc(FRAME_BAUD, &115_200u32.to_le_bytes());
            thread::sleep(Duration::from_millis(60));
            if let Some(p) = self.port.as_mut() {
                let _ = p.set_baud_rate(115_200);
            }
            self.rx_buf.clear();
            let _ = self.rpc(FRAME_HELLO, HELLO_PAYLOAD);
            self.fast_baud = false;
        }
        self.port = None;
    }

    pub fn set_dial_hz(&mut self, hz: u64) -> Result<(), String> {
        let target = dial_to_band_slot(hz);
        let delta = target - self.band_slot;
        self.dial_hz = hz;
        if delta != 0 && self.port.is_some() {
            let was_audio = self.audio_on;
            if was_audio {
                let _ = self.rpc(FRAME_AUDIO, &[0]);
                self.audio_on = false;
            }
            let _ = self.send_knob(delta.clamp(-4, 4) as i8);
            self.band_slot = target;
            if was_audio {
                let _ = self.start_audio();
            }
        }
        Ok(())
    }

    pub fn drain_idle(
        &mut self,
        duration: Duration,
        running: &AtomicBool,
        on_chunk: &mut dyn FnMut(&[i16]),
    ) {
        if self.port.is_none() {
            thread::sleep(duration);
            return;
        }
        let end = Instant::now() + duration;
        let mut pcm = Vec::with_capacity(512);
        while running.load(Ordering::Relaxed) && Instant::now() < end {
            self.poll_audio_frames(&mut pcm, None);
            if !pcm.is_empty() {
                on_chunk(&pcm);
                pcm.clear();
            } else {
                thread::sleep(Duration::from_millis(5));
            }
        }
    }

    pub fn capture_12k_streaming(
        &mut self,
        duration_sec: f64,
        running: &AtomicBool,
        on_chunk: &mut dyn FnMut(&[i16]),
    ) -> Result<(Vec<i16>, f32), String> {
        if self.port.is_none() {
            self.connect()?;
        }
        if !self.audio_on {
            self.start_audio()?;
        }
        let target_samples = (duration_sec * 12_000.0).round() as usize;
        let mut all = Vec::with_capacity(target_samples + 1024);
        let mut chunk = Vec::with_capacity(512);
        let mut peak_abs: i32 = 0;
        let deadline = Instant::now() + Duration::from_secs_f64(duration_sec + 0.15);

        while running.load(Ordering::Relaxed)
            && all.len() < target_samples
            && Instant::now() < deadline
        {
            self.poll_audio_frames(&mut chunk, Some(&mut peak_abs));
            if !chunk.is_empty() {
                on_chunk(&chunk);
                all.extend_from_slice(&chunk);
                chunk.clear();
            } else {
                thread::sleep(Duration::from_millis(4));
            }
        }
        if all.len() < target_samples {
            all.resize(target_samples, 0);
        } else if all.len() > target_samples {
            all.truncate(target_samples);
        }
        Ok((all, (peak_abs as f32 / 32768.0).clamp(0.0, 1.0)))
    }

    pub fn transmit_burst(
        &mut self,
        tones: &[u8; urad950::N_SYMBOLS],
        audio_hz: u16,
        running: &AtomicBool,
    ) -> Result<(), String> {
        if self.cfg.dry_tx {
            let end = Instant::now() + Duration::from_millis(8_800);
            while running.load(Ordering::Relaxed) && Instant::now() < end {
                thread::sleep(Duration::from_millis(25));
            }
            return Ok(());
        }
        // Trigger an immediate 110-symbol BK4829 4-FSK burst on the RT-950 Pro via Link `RAD950_TX` (`0x65`).
        if self.port.is_some() {
            let was_audio = self.audio_on;
            if was_audio {
                let _ = self.rpc(FRAME_AUDIO, &[0]);
                self.audio_on = false;
            }
            let rf_freq_10hz = ((self.dial_hz + audio_hz as u64) / 10) as u32;
            const PACKED: usize = urad950::N_SYMBOLS.div_ceil(4); // 28 bytes
            let mut payload = [0u8; 5 + PACKED];
            payload[..4].copy_from_slice(&rf_freq_10hz.to_le_bytes());
            payload[4] = 0xFF; // use radio's configured TX power
            for (i, &t) in tones.iter().enumerate() {
                payload[5 + (i >> 2)] |= (t & 0x03) << ((i & 3) * 2);
            }
            self.rpc(FRAME_RAD950_TX, &payload)
                .map_err(|e| format!("RT-950 RAD950_TX failed: {e}"))?;
            // Wait for the 8.80 s burst + unkey + SI4732 RX restore to finish (UART TX pauses while keyed)
            let end = Instant::now() + Duration::from_millis(9_050);
            while running.load(Ordering::Relaxed) && Instant::now() < end {
                thread::sleep(Duration::from_millis(30));
            }
            self.rx_buf.clear();
            if was_audio {
                let _ = self.start_audio();
            }
        }
        Ok(())
    }

    fn start_audio(&mut self) -> Result<(), String> {
        self.rpc(FRAME_AUDIO, &[1])?;
        self.audio_on = true;
        self.last_audio_renew = Instant::now();
        Ok(())
    }

    fn send_key(&mut self, code: u8) -> Result<(), String> {
        self.rpc(FRAME_KEY, &[code, KEY_SHORT]).map(|_| ())
    }

    fn send_knob(&mut self, delta: i8) -> Result<(), String> {
        self.rpc(FRAME_KEY, &[KEY_KNOB, delta as u8]).map(|_| ())
    }

    fn next_seq(&mut self) -> u8 {
        self.seq = self.seq.wrapping_add(1);
        if self.seq == 0 {
            self.seq = 1;
        }
        self.seq
    }

    fn send_frame(&mut self, kind: u8, payload: &[u8]) -> Result<u8, String> {
        let seq = self.next_seq();
        let mut raw = Vec::with_capacity(payload.len() + 4);
        raw.push(kind);
        raw.push(seq);
        raw.extend_from_slice(payload);
        let crc = crc16_ccitt(&raw);
        raw.push((crc & 0xFF) as u8);
        raw.push((crc >> 8) as u8);
        let mut encoded = cobs_encode(&raw);
        encoded.push(0x00);
        let p = self.port.as_mut().ok_or_else(|| "serial port closed".to_string())?;
        p.write_all(&encoded).map_err(|e| format!("serial write: {e}"))?;
        let _ = p.flush();
        Ok(seq)
    }

    fn rpc(&mut self, kind: u8, payload: &[u8]) -> Result<Vec<u8>, String> {
        let seq = self.send_frame(kind, payload)?;
        let want_type = kind | 0x80;
        let deadline = Instant::now() + Duration::from_millis(1200);
        while Instant::now() < deadline {
            self.read_into_buf();
            while let Some(frame) = self.take_decoded_frame() {
                if frame.len() >= 3 && frame[0] == want_type && frame[1] == seq {
                    let status = frame[2];
                    if status != 0 {
                        return Err(format!("ferrite link status {status}"));
                    }
                    return Ok(frame[3..].to_vec());
                }
            }
            thread::sleep(Duration::from_millis(3));
        }
        Err(format!("timeout waiting for Ferrite link reply 0x{want_type:02x}"))
    }

    fn poll_audio_frames(&mut self, out_pcm: &mut Vec<i16>, mut peak_abs: Option<&mut i32>) {
        if self.audio_on && self.last_audio_renew.elapsed() >= Duration::from_secs(3) {
            let _ = self.send_frame(FRAME_AUDIO, &[1]);
            self.last_audio_renew = Instant::now();
        }
        self.read_into_buf();
        while let Some(frame) = self.take_decoded_frame() {
            if frame.len() >= 6 && frame[0] == FRAME_AUDIO_DATA {
                // frame[0]=type, frame[1]=seq, frame[2..6]=audio_seq(u16)+slot(u8)+flags(u8), frame[6..]=ulaw
                let ulaw = &frame[6..];
                if ulaw.is_empty() {
                    // Keepalive while squelched -> 16 ms of silence (192 samples @ 12 kHz)
                    out_pcm.resize(out_pcm.len() + 192, 0);
                } else {
                    for &b in ulaw {
                        let s = ulaw_to_i16(b);
                        if let Some(pk) = peak_abs.as_deref_mut() {
                            let a = (s as i32).abs();
                            if a > *pk {
                                *pk = a;
                            }
                        }
                        out_pcm.push(s);
                    }
                }
            }
        }
    }

    fn read_into_buf(&mut self) {
        let Some(p) = self.port.as_mut() else { return };
        let mut tmp = [0u8; 1024];
        match p.read(&mut tmp) {
            Ok(n) if n > 0 => self.rx_buf.extend_from_slice(&tmp[..n]),
            _ => {}
        }
    }

    fn take_decoded_frame(&mut self) -> Option<Vec<u8>> {
        while let Some(pos) = self.rx_buf.iter().position(|&b| b == 0x00) {
            let chunk: Vec<u8> = self.rx_buf.drain(..=pos).collect();
            let encoded = &chunk[..chunk.len() - 1];
            if encoded.is_empty() {
                continue;
            }
            let Ok(raw) = cobs_decode(encoded) else {
                continue;
            };
            if raw.len() < 4 {
                continue;
            }
            let body = &raw[..raw.len() - 2];
            let got_crc = u16::from_le_bytes([raw[raw.len() - 2], raw[raw.len() - 1]]);
            if crc16_ccitt(body) == got_crc {
                return Some(body.to_vec());
            }
        }
        None
    }
}

fn dial_to_band_slot(dial_hz: u64) -> i32 {
    if dial_hz < 20_000_000 {
        0 // 17m
    } else if dial_hz < 23_000_000 {
        1 // 15m
    } else if dial_hz < 26_500_000 {
        2 // 12m
    } else {
        3 // 10m
    }
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if (crc & 0x8000) != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn cobs_encode(src: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(src.len() + 2);
    out.push(0);
    let mut code_at = 0usize;
    let mut code = 1u8;
    for &b in src {
        if b == 0 {
            out[code_at] = code;
            code_at = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(b);
            code += 1;
            if code == 0xFF {
                out[code_at] = code;
                code_at = out.len();
                out.push(0);
                code = 1;
            }
        }
    }
    out[code_at] = code;
    out
}

fn cobs_decode(src: &[u8]) -> Result<Vec<u8>, ()> {
    let mut out = Vec::with_capacity(src.len());
    let mut i = 0usize;
    while i < src.len() {
        let code = src[i] as usize;
        if code == 0 || i + code > src.len() {
            return Err(());
        }
        out.extend_from_slice(&src[i + 1..i + code]);
        i += code;
        if code < 0xFF && i < src.len() {
            out.push(0);
        }
    }
    Ok(out)
}

/// Standard ITU-T G.711 µ-law decode (`u8` -> `i16`).
#[inline]
fn ulaw_to_i16(u: u8) -> i16 {
    let u = !u;
    let sign = (u & 0x80) != 0;
    let exp = ((u >> 4) & 0x07) as u32;
    let mant = (u & 0x0F) as i32;
    let t = ((mant << 3) + 0x84) << exp;
    let sample = t - 0x84;
    if sign { -(sample as i16) } else { sample as i16 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cobs_and_crc_roundtrip() {
        let msg = [0x01, 0x02, 0x00, 0x56, 0x01, 0x00, 0xFF];
        let enc = cobs_encode(&msg);
        assert!(!enc.contains(&0));
        assert_eq!(cobs_decode(&enc).unwrap(), msg);
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn ulaw_silence_and_symmetry() {
        assert_eq!(ulaw_to_i16(0xFF), 0);
        assert_eq!(ulaw_to_i16(0x7F), 0);
        assert_eq!(ulaw_to_i16(0x00), -ulaw_to_i16(0x80));
    }
}
