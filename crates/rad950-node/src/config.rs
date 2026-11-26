//! Persistent cross-platform TOML settings (`config.toml`).
//!
//! Location:
//! - macOS: `~/Library/Application Support/rad950/config.toml`
//! - Linux: `~/.config/rad950/config.toml`
//! - Windows: `%APPDATA%\rad950\config.toml`

use std::fs;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use crate::audio::AudioConfig;
use crate::engine::{AutoAnswer, Band};
use crate::hackrf::HackrfConfig;
use crate::hamlib::HamlibConfig;
use crate::rt950::Rt950Config;
use crate::runner::{BackendKind, NodeConfig};
use crate::tci::TciConfig;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SavedConfig {
    pub call: String,
    pub grid: String,
    pub band: String,
    pub dial_hz: Option<u64>,
    pub subch: usize,
    pub mode60: bool,
    pub auto_answer: String,
    pub rdv_follow: bool,
    pub rdv_call_listen: bool,
    pub dry_tx: bool,
    pub tx_watchdog_mins: u32,
    pub alert_tones: bool,
    pub cw_id: bool,

    /// `"audio"`, `"tci"`, or `"hackrf"`
    pub backend: String,

    // Audio + Hamlib
    pub audio_in: String,
    pub audio_out: String,
    pub tx_audio_gain: f32,
    pub rigctld_addr: String,
    pub rig_model: Option<u32>,
    pub rig_port: String,
    pub rig_baud: Option<u32>,

    // TCI
    pub tci_addr: String,

    // HackRF
    pub hackrf_serial: String,
    pub lna_db: u32,
    pub vga_db: u32,
    pub tx_vga_db: u32,
    pub ppm: f64,

    // Logging & Spotting
    pub log_path: String,
    pub all_txt_path: String,
    pub auto_upload_qso: bool,
    pub spot_enabled: bool,
    pub spot_url: String,
    pub spot_key: String,

    // Web Remote Control (rad950.org)
    pub web_remote_enabled: bool,
    pub web_remote_allow_tx: bool,
    pub web_remote_station_name: String,
    pub web_remote_station_id: String,

    // Serial-connected RT-950 Pro (Ferrite Link v1)
    pub rt950_port: String,
}

fn entropy_u64() -> u64 {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    use std::time::{SystemTime, UNIX_EPOCH};
    let mut h = RandomState::new().build_hasher();
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    h.write_u128(nanos);
    h.write_u32(std::process::id());
    h.finish()
}

pub fn generate_station_name() -> String {
    const WORDS: &[&str] = &[
        "dipole", "yagi", "vertical", "hexbeam", "endfed", "moxon", "delta", "quad",
        "zepp", "rhombic", "beverage", "shack", "beacon", "portable", "summit", "field",
    ];
    let r = entropy_u64();
    let word = WORDS[(r as usize) % WORDS.len()];
    let num = ((r >> 16) % 9000) + 1000;
    format!("{word}-{num}")
}

pub fn generate_station_id() -> String {
    let a = entropy_u64();
    let b = entropy_u64().rotate_left(17) ^ 0x9e37_79b9_7f4a_7c15;
    format!("{:012x}", (a ^ b) & 0xffff_ffff_ffff)
}

impl Default for SavedConfig {
    fn default() -> Self {
        let default_log = default_config_dir()
            .join("rad950_log.adi")
            .to_string_lossy()
            .into_owned();
        let default_all_txt = default_config_dir()
            .join("ALL.TXT")
            .to_string_lossy()
            .into_owned();
        Self {
            call: "N0CAL".to_string(),
            grid: "----".to_string(),
            band: "17m".to_string(),
            dial_hz: None,
            subch: 4,
            mode60: false,
            auto_answer: "off".to_string(),
            rdv_follow: false,
            rdv_call_listen: false,
            dry_tx: true,
            tx_watchdog_mins: 6,
            alert_tones: true,
            cw_id: false,
            backend: "audio".to_string(),
            audio_in: String::new(),
            audio_out: String::new(),
            tx_audio_gain: 0.5,
            rigctld_addr: String::new(),
            rig_model: None,
            rig_port: String::new(),
            rig_baud: None,
            tci_addr: "127.0.0.1:50001".to_string(),
            hackrf_serial: String::new(),
            lna_db: 24,
            vga_db: 20,
            tx_vga_db: 20,
            ppm: 0.0,
            log_path: default_log,
            all_txt_path: default_all_txt,
            auto_upload_qso: true,
            spot_enabled: true,
            spot_url: "https://rad950.org/api/spots".to_string(),
            spot_key: String::new(),
            web_remote_enabled: false,
            web_remote_allow_tx: false,
            web_remote_station_name: generate_station_name(),
            web_remote_station_id: generate_station_id(),
            rt950_port: String::new(),
        }
    }
}

pub fn default_config_dir() -> PathBuf {
    dirs::config_dir()
        .unwrap_or_else(|| PathBuf::from("."))
        .join("rad950")
}

pub fn default_config_path() -> PathBuf {
    default_config_dir().join("config.toml")
}

impl SavedConfig {
    pub fn load(path: &PathBuf) -> Self {
        let mut cfg: Self = match fs::read_to_string(path) {
            Ok(text) => toml::from_str(&text).unwrap_or_default(),
            Err(_) => Self::default(),
        };
        if cfg.call.trim().eq_ignore_ascii_case("N0CALL") {
            cfg.call = "N0CAL".to_string();
        }
        if cfg.spot_url.trim() == "https://rad950.com/api/spots" {
            cfg.spot_url = "https://rad950.org/api/spots".to_string();
        }
        if cfg.web_remote_station_name.trim().is_empty() {
            cfg.web_remote_station_name = generate_station_name();
        }
        if cfg.web_remote_station_id.trim().is_empty() {
            cfg.web_remote_station_id = generate_station_id();
        }
        cfg
    }

    pub fn save(&self, path: &PathBuf) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| format!("create config dir: {e}"))?;
        }
        let body = toml::to_string_pretty(self).map_err(|e| format!("serialize config: {e}"))?;
        fs::write(path, body).map_err(|e| format!("write {}: {e}", path.display()))
    }

    pub fn to_node_config(&self) -> Result<NodeConfig, String> {
        let dial_hz = self.dial_hz.unwrap_or_else(|| {
            Band::parse(&self.band)
                .map(|(_, hz)| hz)
                .unwrap_or(18_104_000)
        });
        let auto_answer = AutoAnswer::parse(&self.auto_answer).unwrap_or(AutoAnswer::Off);
        let backend = match self.backend.trim().to_ascii_lowercase().as_str() {
            "tci" => BackendKind::Tci(TciConfig {
                addr: self.tci_addr.clone(),
                tx_gain: self.tx_audio_gain.clamp(0.05, 0.95),
                dry_tx: self.dry_tx,
            }),
            "rt950" => BackendKind::Rt950(Rt950Config {
                port: self.rt950_port.trim().to_string(),
                dry_tx: self.dry_tx,
            }),
            "hackrf" => BackendKind::Hackrf(HackrfConfig {
                serial: if self.hackrf_serial.trim().is_empty() {
                    None
                } else {
                    Some(self.hackrf_serial.trim().to_string())
                },
                lna_db: self.lna_db,
                vga_db: self.vga_db,
                tx_vga_db: self.tx_vga_db,
                ppm: self.ppm,
                dry_tx: self.dry_tx,
            }),
            _ => {
                let addr = if self.rigctld_addr.trim().is_empty() && self.rig_model.is_some() {
                    "127.0.0.1:4532".to_string()
                } else {
                    self.rigctld_addr.trim().to_string()
                };
                BackendKind::Audio(AudioConfig {
                    input_device: if self.audio_in.trim().is_empty() {
                        None
                    } else {
                        Some(self.audio_in.trim().to_string())
                    },
                    output_device: if self.audio_out.trim().is_empty() {
                        None
                    } else {
                        Some(self.audio_out.trim().to_string())
                    },
                    tx_gain: self.tx_audio_gain.clamp(0.05, 0.95),
                    dry_tx: self.dry_tx,
                    hamlib: HamlibConfig {
                        addr,
                        spawn_model: self.rig_model,
                        spawn_port: if self.rig_port.trim().is_empty() {
                            None
                        } else {
                            Some(self.rig_port.trim().to_string())
                        },
                        spawn_baud: self.rig_baud,
                    },
                })
            }
        };

        Ok(NodeConfig {
            call: self.call.clone(),
            grid: self.grid.clone(),
            dial_hz,
            subch: self.subch,
            mode60: self.mode60,
            auto_answer,
            rdv_follow: self.rdv_follow,
            rdv_call_listen: self.rdv_call_listen,
            tx_watchdog_mins: self.tx_watchdog_mins,
            alert_tones: self.alert_tones,
            cw_id: self.cw_id,
            log_path: PathBuf::from(&self.log_path),
            all_txt_path: PathBuf::from(&self.all_txt_path),
            auto_upload_qso: self.auto_upload_qso,
            spot_enabled: self.spot_enabled,
            spot_url: self.spot_url.clone(),
            spot_key: self.spot_key.clone(),
            web_remote_enabled: self.web_remote_enabled,
            web_remote_allow_tx: self.web_remote_allow_tx,
            web_remote_station_name: self.web_remote_station_name.clone(),
            web_remote_station_id: self.web_remote_station_id.clone(),
            backend,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saved_config_round_trip() {
        let mut c = SavedConfig::default();
        c.call = "NQ6N".to_string();
        c.grid = "EN61".to_string();
        c.backend = "tci".to_string();
        c.tci_addr = "192.168.0.55:50001".to_string();
        let s = toml::to_string_pretty(&c).unwrap();
        let d: SavedConfig = toml::from_str(&s).unwrap();
        assert_eq!(d.call, "NQ6N");
        assert_eq!(d.grid, "EN61");
        assert_eq!(d.backend, "tci");
        assert_eq!(d.tci_addr, "192.168.0.55:50001");
    }

    #[test]
    fn default_config_parses_for_first_launch() {
        let def = SavedConfig::default();
        let node = def.to_node_config().expect("default SavedConfig -> NodeConfig");
        let (_, _, disp) = crate::engine::parse_call_and_suffix(&node.call)
            .expect("default callsign must be valid RAD950 callsign");
        assert_eq!(disp, "N0CAL");
        let (_, grid) = crate::engine::parse_grid(&node.grid)
            .expect("default grid must be valid");
        assert_eq!(grid, "----");
    }
}
