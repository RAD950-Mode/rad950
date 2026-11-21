//! Cross-platform Hamlib CAT/PTT client via `rigctld` TCP protocol
//! (works identically on macOS, Linux, and Windows; supports external `rigctld` or auto-spawning one).

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct HamlibConfig {
    /// `host:port` of `rigctld` (e.g. `"127.0.0.1:4532"`). Empty disables CAT.
    pub addr: String,
    /// Optional Hamlib rig model ID (e.g. `3073` for IC-7300) to auto-spawn `rigctld`.
    pub spawn_model: Option<u32>,
    /// Optional serial device path (`COM3`, `/dev/ttyUSB0`, `/dev/cu.usbserial-...`) for auto-spawn.
    pub spawn_port: Option<String>,
    /// Optional serial baud rate for auto-spawn.
    pub spawn_baud: Option<u32>,
}

impl Default for HamlibConfig {
    fn default() -> Self {
        Self {
            addr: String::new(),
            spawn_model: None,
            spawn_port: None,
            spawn_baud: None,
        }
    }
}

pub struct HamlibClient {
    cfg: HamlibConfig,
    stream: Option<TcpStream>,
    child: Option<Child>,
}

impl HamlibClient {
    pub fn new(cfg: HamlibConfig) -> Self {
        Self {
            cfg,
            stream: None,
            child: None,
        }
    }

    pub fn is_configured(&self) -> bool {
        !self.cfg.addr.trim().is_empty()
    }

    pub fn start_if_needed(&mut self) -> Result<(), String> {
        if !self.is_configured() {
            return Ok(());
        }
        if let (Some(model), Some(port)) = (self.cfg.spawn_model, self.cfg.spawn_port.as_deref()) {
            if self.child.is_none() {
                let tcp_port = self
                    .cfg
                    .addr
                    .rsplit_once(':')
                    .map(|(_, p)| p)
                    .unwrap_or("4532");
                let mut cmd = Command::new("rigctld");
                cmd.args(["-m", &model.to_string(), "-r", port, "-t", tcp_port]);
                if let Some(baud) = self.cfg.spawn_baud {
                    cmd.args(["-s", &baud.to_string()]);
                }
                cmd.stdout(Stdio::null()).stderr(Stdio::null());
                let child = cmd
                    .spawn()
                    .map_err(|e| format!("failed to spawn rigctld: {e}"))?;
                self.child = Some(child);
                std::thread::sleep(Duration::from_millis(400));
            }
        }
        self.ensure_connected()
    }

    fn ensure_connected(&mut self) -> Result<(), String> {
        if self.stream.is_some() {
            return Ok(());
        }
        let s = TcpStream::connect(self.cfg.addr.trim())
            .map_err(|e| format!("rigctld connect {} failed: {e}", self.cfg.addr))?;
        let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
        let _ = s.set_write_timeout(Some(Duration::from_secs(2)));
        self.stream = Some(s);
        Ok(())
    }

    fn send_cmd(&mut self, line: &str) -> Result<String, String> {
        if !self.is_configured() {
            return Ok(String::new());
        }
        self.ensure_connected()?;
        let res = (|| -> Result<String, String> {
            let s = self.stream.as_mut().unwrap();
            writeln!(s, "{line}").map_err(|e| e.to_string())?;
            s.flush().map_err(|e| e.to_string())?;
            let mut reader = BufReader::new(s.try_clone().map_err(|e| e.to_string())?);
            let mut resp = String::new();
            reader.read_line(&mut resp).map_err(|e| e.to_string())?;
            Ok(resp.trim().to_string())
        })();
        if res.is_err() {
            self.stream = None;
        }
        res
    }

    pub fn set_freq(&mut self, hz: u64) -> Result<(), String> {
        if !self.is_configured() {
            return Ok(());
        }
        let resp = self.send_cmd(&format!("F {hz}"))?;
        if resp.starts_with("RPRT ") && !resp.starts_with("RPRT 0") {
            return Err(format!("rigctld set_freq error: {resp}"));
        }
        Ok(())
    }

    pub fn set_mode_usb(&mut self) -> Result<(), String> {
        if !self.is_configured() {
            return Ok(());
        }
        let resp = self.send_cmd("M PKTUSB 0")?;
        if resp.starts_with("RPRT ") && !resp.starts_with("RPRT 0") {
            let resp2 = self.send_cmd("M USB 0")?;
            if resp2.starts_with("RPRT ") && !resp2.starts_with("RPRT 0") {
                return Err(format!("rigctld set_mode error: {resp2}"));
            }
        }
        Ok(())
    }

    pub fn set_ptt(&mut self, on: bool) -> Result<(), String> {
        if !self.is_configured() {
            return Ok(());
        }
        let resp = self.send_cmd(&format!("T {}", if on { 1 } else { 0 }))?;
        if resp.starts_with("RPRT ") && !resp.starts_with("RPRT 0") {
            return Err(format!("rigctld set_ptt error: {resp}"));
        }
        Ok(())
    }
}

impl Drop for HamlibClient {
    fn drop(&mut self) {
        let _ = self.set_ptt(false);
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}
