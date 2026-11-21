//! Cross-platform TCI v1.8 client backend (WebSocket CAT + 48 kHz stereo float32 audio)
//! compatible with `tci_hackrf_bridge.py`, ExpertSDR, Thetis, and SunSDR.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use urad950::N_SYMBOLS;

use crate::audio::synth_4fsk_f32;

const TCI_HDR_BYTES: usize = 64;
const STREAM_RX_AUDIO: u32 = 1;
const STREAM_TX_AUDIO: u32 = 2;
const STREAM_TX_CHRONO: u32 = 3;

#[derive(Clone, Debug)]
pub struct TciConfig {
    /// `host:port` (e.g. `"192.168.0.55:50001"` or `"ws://192.168.0.55:50001"`).
    pub addr: String,
    pub tx_gain: f32,
    pub dry_tx: bool,
}

impl Default for TciConfig {
    fn default() -> Self {
        Self {
            addr: "127.0.0.1:50001".to_string(),
            tx_gain: 0.85,
            dry_tx: true,
        }
    }
}

pub struct TciBackend {
    cfg: TciConfig,
    dial_hz: u64,
    stream: Option<TcpStream>,
    mask_seed: u32,
}

impl TciBackend {
    pub fn new(cfg: TciConfig, dial_hz: u64) -> Self {
        Self {
            cfg,
            dial_hz,
            stream: None,
            mask_seed: 0x1234_5678,
        }
    }

    fn clean_addr(&self) -> String {
        let s = self.cfg.addr.trim();
        let s = s.strip_prefix("ws://").unwrap_or(s);
        let s = s.strip_prefix("wss://").unwrap_or(s);
        s.split('/').next().unwrap_or(s).to_string()
    }

    pub fn connect(&mut self) -> Result<(), String> {
        if self.stream.is_some() {
            return Ok(());
        }
        let addr = self.clean_addr();
        let mut s = TcpStream::connect(&addr)
            .map_err(|e| format!("TCI connect {addr} failed: {e}"))?;
        let _ = s.set_nodelay(true);
        let _ = s.set_read_timeout(Some(Duration::from_millis(250)));
        let _ = s.set_write_timeout(Some(Duration::from_secs(2)));

        let req = format!(
            "GET / HTTP/1.1\r\n\
             Host: {addr}\r\n\
             Upgrade: websocket\r\n\
             Connection: Upgrade\r\n\
             Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\
             Sec-WebSocket-Version: 13\r\n\r\n"
        );
        s.write_all(req.as_bytes()).map_err(|e| e.to_string())?;
        s.flush().map_err(|e| e.to_string())?;

        // Read HTTP 101 response header byte-by-byte so we never over-read WebSocket frames
        let mut hdr = Vec::with_capacity(256);
        let mut byte = [0u8; 1];
        let deadline = Instant::now() + Duration::from_secs(4);
        while Instant::now() < deadline {
            match s.read(&mut byte) {
                Ok(0) => return Err("TCI closed during handshake".to_string()),
                Ok(_) => {
                    hdr.push(byte[0]);
                    if hdr.ends_with(b"\r\n\r\n") {
                        break;
                    }
                    if hdr.len() > 4096 {
                        return Err("TCI HTTP header too large".to_string());
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(format!("TCI handshake read error: {e}")),
            }
        }
        let hdr_str = String::from_utf8_lossy(&hdr);
        if !hdr_str.contains("101") {
            return Err(format!("TCI upgrade rejected: {}", hdr_str.lines().next().unwrap_or("")));
        }

        self.stream = Some(s);

        // Drain initial state messages up to `ready;`
        let ready_deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < ready_deadline {
            match self.read_ws_frame() {
                Ok(Some((0x1, pay))) => {
                    if String::from_utf8_lossy(&pay).contains("ready;") {
                        break;
                    }
                }
                Ok(Some((0x9, pay))) => {
                    let _ = self.send_ws_frame(0xA, &pay);
                }
                Ok(Some(_)) => {}
                Ok(None) => continue,
                Err(e) => {
                    self.stream = None;
                    return Err(e);
                }
            }
        }

        let dial = self.dial_hz;
        self.send_text("modulation:0,digu;")?;
        self.send_text(&format!("vfo:0,0,{dial};"))?;
        self.send_text("audio_start:0;")?;
        Ok(())
    }

    pub fn set_dial_hz(&mut self, hz: u64) -> Result<(), String> {
        self.dial_hz = hz;
        if self.stream.is_some() {
            if let Err(e) = self.send_text(&format!("vfo:0,0,{hz};")) {
                self.stream = None;
                return Err(e);
            }
        }
        Ok(())
    }

    pub fn set_dry_tx(&mut self, dry_tx: bool) {
        self.cfg.dry_tx = dry_tx;
    }

    fn next_mask(&mut self) -> [u8; 4] {
        let mut x = self.mask_seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.mask_seed = x;
        x.to_le_bytes()
    }

    fn send_ws_frame(&mut self, opcode: u8, payload: &[u8]) -> Result<(), String> {
        let mask = self.next_mask();
        let n = payload.len();
        let mut frame = Vec::with_capacity(14 + n);
        frame.push(0x80 | (opcode & 0x0F));
        if n < 126 {
            frame.push(0x80 | (n as u8));
        } else if n < 65_536 {
            frame.push(0x80 | 126);
            frame.extend_from_slice(&(n as u16).to_be_bytes());
        } else {
            frame.push(0x80 | 127);
            frame.extend_from_slice(&(n as u64).to_be_bytes());
        }
        frame.extend_from_slice(&mask);
        for (i, &b) in payload.iter().enumerate() {
            frame.push(b ^ mask[i & 3]);
        }
        let s = self.stream.as_mut().ok_or("TCI not connected")?;
        s.write_all(&frame).map_err(|e| format!("TCI write error: {e}"))?;
        s.flush().map_err(|e| format!("TCI flush error: {e}"))?;
        Ok(())
    }

    fn send_text(&mut self, text: &str) -> Result<(), String> {
        self.send_ws_frame(0x1, text.as_bytes())
    }

    /// Returns `Ok(None)` on read timeout (no frame ready yet).
    fn read_ws_frame(&mut self) -> Result<Option<(u8, Vec<u8>)>, String> {
        let s = self.stream.as_mut().ok_or("TCI not connected")?;
        let mut hdr = [0u8; 2];
        match s.read(&mut hdr[..1]) {
            Ok(0) => return Err("TCI connection closed".to_string()),
            Ok(1) => {}
            Ok(_) => unreachable!(),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => {
                return Ok(None);
            }
            Err(e) => return Err(format!("TCI read error: {e}")),
        }
        read_exact_retry(s, &mut hdr[1..2])?;
        let opcode = hdr[0] & 0x0F;
        let masked = (hdr[1] & 0x80) != 0;
        let mut len = (hdr[1] & 0x7F) as usize;
        if len == 126 {
            let mut b = [0u8; 2];
            read_exact_retry(s, &mut b)?;
            len = u16::from_be_bytes(b) as usize;
        } else if len == 127 {
            let mut b = [0u8; 8];
            read_exact_retry(s, &mut b)?;
            len = u64::from_be_bytes(b) as usize;
        }
        let mut mask = [0u8; 4];
        if masked {
            read_exact_retry(s, &mut mask)?;
        }
        let mut payload = vec![0u8; len];
        if len > 0 {
            read_exact_retry(s, &mut payload)?;
        }
        if masked {
            for (i, b) in payload.iter_mut().enumerate() {
                *b ^= mask[i & 3];
            }
        }
        Ok(Some((opcode, payload)))
    }

    pub fn capture_12k(&mut self, duration_sec: f64, running: &AtomicBool) -> Result<(Vec<i16>, f32), String> {
        self.capture_12k_streaming(duration_sec, running, &mut |_| {})
    }

    pub fn capture_12k_streaming(
        &mut self,
        duration_sec: f64,
        running: &AtomicBool,
        on_chunk: &mut dyn FnMut(&[i16]),
    ) -> Result<(Vec<i16>, f32), String> {
        self.connect()?;
        let need_12k = (12_000.0 * duration_sec).round() as usize;
        let mut out = Vec::with_capacity(need_12k + 1024);
        let mut peak = 0.0f32;
        let deadline = Instant::now() + Duration::from_secs_f64(duration_sec + 1.5);

        while running.load(Ordering::Relaxed) && out.len() < need_12k && Instant::now() < deadline {
            match self.read_ws_frame() {
                Ok(Some((0x2, pay))) if pay.len() > TCI_HDR_BYTES => {
                    let length = u32::from_le_bytes(pay[20..24].try_into().unwrap()) as usize;
                    let stype = u32::from_le_bytes(pay[24..28].try_into().unwrap());
                    if stype == STREAM_RX_AUDIO {
                        let avail_floats = ((pay.len() - TCI_HDR_BYTES) / 4).min(length);
                        // Stereo interleaved (L, R) @ 48 kHz -> decimate x4 (4 stereo frames = 8 floats -> 1 sample @ 12 kHz)
                        let n_blocks = avail_floats / 8;
                        let body = &pay[TCI_HDR_BYTES..];
                        let prev_len = out.len();
                        for b in 0..n_blocks {
                            let mut sum = 0.0f32;
                            for k in 0..4 {
                                let off = (b * 8 + k * 2) * 4;
                                let s = f32::from_le_bytes(body[off..off + 4].try_into().unwrap());
                                sum += s;
                                let a = s.abs();
                                if a > peak {
                                    peak = a;
                                }
                            }
                            let avg = sum * 0.25;
                            out.push((avg * 32767.0).clamp(-32767.0, 32767.0) as i16);
                        }
                        if out.len() > prev_len {
                            on_chunk(&out[prev_len..]);
                        }
                    }
                }
                Ok(Some((0x8, _))) => {
                    self.stream = None;
                    return Err("TCI server closed connection".to_string());
                }
                Ok(Some((0x9, pay))) => {
                    let _ = self.send_ws_frame(0xA, &pay);
                }
                Ok(_) => {}
                Err(e) => {
                    self.stream = None;
                    return Err(e);
                }
            }
        }
        out.truncate(need_12k);
        Ok((out, (peak * 100.0).min(100.0)))
    }

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

        self.connect()?;
        let mut wave = synth_4fsk_f32(tones, audio_hz as f64, 48_000, self.cfg.tx_gain.clamp(0.05, 0.95));
        // Pad 160 ms of trailing silence so the last symbol flushes cleanly through the SDR pipe
        wave.resize(wave.len() + 7680, 0.0);

        self.send_text("trx:0,true;")?;
        let mut cursor = 0usize;
        let deadline = Instant::now() + Duration::from_secs(11);
        let mut tx_err: Option<String> = None;

        while running.load(Ordering::Relaxed) && cursor < wave.len() && Instant::now() < deadline {
            match self.read_ws_frame() {
                Ok(Some((0x2, pay))) if pay.len() >= TCI_HDR_BYTES => {
                    let req_floats = u32::from_le_bytes(pay[20..24].try_into().unwrap()) as usize;
                    let stype = u32::from_le_bytes(pay[24..28].try_into().unwrap());
                    if stype == STREAM_TX_CHRONO {
                        let n_frames = (req_floats / 2).max(1);
                        let mut pkt = Vec::with_capacity(TCI_HDR_BYTES + n_frames * 8);
                        pkt.extend_from_slice(&pack_tci_header(0, 48_000, 3, (n_frames * 2) as u32, STREAM_TX_AUDIO));
                        for _ in 0..n_frames {
                            let s = if cursor < wave.len() {
                                let v = wave[cursor];
                                cursor += 1;
                                v
                            } else {
                                0.0
                            };
                            let b = s.to_le_bytes();
                            pkt.extend_from_slice(&b);
                            pkt.extend_from_slice(&b);
                        }
                        if let Err(e) = self.send_ws_frame(0x2, &pkt) {
                            tx_err = Some(e);
                            break;
                        }
                    }
                }
                Ok(Some((0x9, pay))) => {
                    let _ = self.send_ws_frame(0xA, &pay);
                }
                Ok(Some((0x8, _))) => {
                    self.stream = None;
                    tx_err = Some("TCI closed during TX".to_string());
                    break;
                }
                Ok(_) => {}
                Err(e) => {
                    self.stream = None;
                    tx_err = Some(e);
                    break;
                }
            }
        }

        let _ = self.send_text("trx:0,false;");
        if let Some(e) = tx_err {
            return Err(e);
        }
        Ok(())
    }
}

fn read_exact_retry(s: &mut TcpStream, mut buf: &mut [u8]) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_secs(3);
    while !buf.is_empty() {
        match s.read(buf) {
            Ok(0) => return Err("TCI unexpected EOF".to_string()),
            Ok(n) => {
                let tmp = buf;
                buf = &mut tmp[n..];
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => {
                if Instant::now() >= deadline {
                    return Err("TCI read timeout".to_string());
                }
            }
            Err(e) => return Err(format!("TCI read error: {e}")),
        }
    }
    Ok(())
}

fn pack_tci_header(receiver: u32, sample_rate: u32, fmt: u32, length: u32, stype: u32) -> [u8; TCI_HDR_BYTES] {
    let mut h = [0u8; TCI_HDR_BYTES];
    h[0..4].copy_from_slice(&receiver.to_le_bytes());
    h[4..8].copy_from_slice(&sample_rate.to_le_bytes());
    h[8..12].copy_from_slice(&fmt.to_le_bytes());
    h[20..24].copy_from_slice(&length.to_le_bytes());
    h[24..28].copy_from_slice(&stype.to_le_bytes());
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tci_header_layout_matches_spec() {
        let h = pack_tci_header(0, 48_000, 3, 2048, STREAM_TX_AUDIO);
        assert_eq!(u32::from_le_bytes(h[0..4].try_into().unwrap()), 0);
        assert_eq!(u32::from_le_bytes(h[4..8].try_into().unwrap()), 48_000);
        assert_eq!(u32::from_le_bytes(h[8..12].try_into().unwrap()), 3);
        assert_eq!(u32::from_le_bytes(h[20..24].try_into().unwrap()), 2048);
        assert_eq!(u32::from_le_bytes(h[24..28].try_into().unwrap()), 2);
    }

    #[test]
    #[ignore = "requires live TCI server (set TCI_TEST_ADDR or default 192.168.0.55:50001)"]
    fn tci_live_rx_smoke() {
        let addr = std::env::var("TCI_TEST_ADDR").unwrap_or_else(|_| "192.168.0.55:50001".to_string());
        let mut tci = TciBackend::new(
            TciConfig {
                addr: addr.clone(),
                tx_gain: 0.85,
                dry_tx: true,
            },
            18_104_000,
        );
        tci.set_dial_hz(18_104_000).expect("set_dial_hz failed");
        let running = AtomicBool::new(true);
        let (samples, rms) = tci.capture_12k(1.0, &running).expect("capture_12k failed");
        assert_eq!(samples.len(), 12_000);
        let peak = samples.iter().map(|&x| (x as i32).abs()).max().unwrap_or(0);
        eprintln!("TCI live RX smoke ({addr}): len={}, peak={peak}, rms={rms:.5}", samples.len());
        assert!(peak > 0, "expected non-zero audio from live TCI server");
    }
}
