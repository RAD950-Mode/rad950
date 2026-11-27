//! Cross-platform Hamlib CAT/PTT client via `rigctld` TCP protocol
//! (works identically on macOS, Linux, and Windows; supports external `rigctld` or auto-spawning one).

use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

#[cfg(windows)]
use std::os::windows::process::CommandExt;

#[cfg(windows)]
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// Locate a usable `rigctld` binary:
/// 1. Bundled right next to the `rad950` executable (Windows installer & portable ZIP)
/// 2. On `PATH` (`rigctld` / `rigctld.exe`)
/// 3. Common Windows ham-radio installs (WSJT-X, WSJT-Z, JTDX, Hamlib)
fn find_rigctld_binary() -> PathBuf {
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            for name in ["rigctld.exe", "rigctld"] {
                let cand = dir.join(name);
                if cand.is_file() {
                    return cand;
                }
            }
        }
    }

    #[cfg(windows)]
    {
        let fixed_candidates = [
            r"C:\WSJT\wsjtx\bin\rigctld-wsjtx.exe",
            r"C:\WSJT\wsjtz\bin\rigctld-wsjtx.exe",
            r"C:\JTDX64\bin\rigctld-jtdx.exe",
            r"C:\Program Files\hamlib\bin\rigctld.exe",
            r"C:\Program Files (x86)\hamlib\bin\rigctld.exe",
        ];
        // Prefer `rigctld` on PATH if present; otherwise fall back to known install folders.
        if let Some(path_var) = std::env::var_os("PATH") {
            for dir in std::env::split_paths(&path_var) {
                for name in ["rigctld.exe", "rigctld-wsjtx.exe"] {
                    let cand = dir.join(name);
                    if cand.is_file() {
                        return cand;
                    }
                }
            }
        }
        for p in fixed_candidates {
            let cand = PathBuf::from(p);
            if cand.is_file() {
                return cand;
            }
        }
        for root in [r"C:\Program Files", r"C:\Program Files (x86)"] {
            if let Ok(rd) = std::fs::read_dir(root) {
                for entry in rd.flatten() {
                    let name = entry.file_name().to_string_lossy().to_ascii_lowercase();
                    if name.starts_with("hamlib") {
                        let cand = entry.path().join("bin").join("rigctld.exe");
                        if cand.is_file() {
                            return cand;
                        }
                    }
                }
            }
        }
    }

    PathBuf::from("rigctld")
}

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
                let rigctld_bin = find_rigctld_binary();
                let mut cmd = Command::new(&rigctld_bin);
                cmd.args(["-m", &model.to_string(), "-r", port, "-t", tcp_port]);
                if let Some(baud) = self.cfg.spawn_baud {
                    cmd.args(["-s", &baud.to_string()]);
                }
                cmd.stdout(Stdio::null()).stderr(Stdio::piped());
                #[cfg(windows)]
                cmd.creation_flags(CREATE_NO_WINDOW);
                let mut child = cmd.spawn().map_err(|e| {
                    format!(
                        "Could not start rigctld ({}): {e}. Reinstall RAD-950 (bundles rigctld) or install Hamlib.",
                        rigctld_bin.display()
                    )
                })?;

                // Poll for up to 1.5s: detect immediate rigctld exit (wrong baud / busy COM port)
                // or connect as soon as rigctld starts listening.
                let deadline = Instant::now() + Duration::from_millis(1500);
                while Instant::now() < deadline {
                    if let Ok(Some(status)) = child.try_wait() {
                        let mut err_txt = String::new();
                        if let Some(mut se) = child.stderr.take() {
                            let _ = se.read_to_string(&mut err_txt);
                        }
                        let snippet = err_txt
                            .lines()
                            .map(str::trim)
                            .filter(|l| !l.is_empty())
                            .last()
                            .unwrap_or("check COM port, baud rate (e.g. 19200/38400/115200), and close WSJT-X/WSJT-Z");
                        return Err(format!("rigctld exited ({status}): {snippet}"));
                    }
                    if self.ensure_connected().is_ok() {
                        self.child = Some(child);
                        return Ok(());
                    }
                    std::thread::sleep(Duration::from_millis(80));
                }
                self.child = Some(child);
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
