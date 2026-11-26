mod assistant;
mod gui;
mod tui;
mod waterfall;

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, ValueEnum};
use rad950_node::audio::list_audio_devices;
use rad950_node::config::{default_config_path, SavedConfig};
use rad950_node::rt950::list_serial_ports;
use rad950_node::runner::NodeRunner;

#[derive(Clone, Copy, Debug, ValueEnum)]
enum CliBackend {
    Audio,
    Tci,
    Hackrf,
    Rt950,
}

const AFTER_HELP: &str = "\
EXAMPLES:
  # List sound cards and serial ports
  rad950 --list-audio
  rad950 --list-serial

  # Standard HF rig via USB sound card + external Hamlib rigctld (17m, live TX)
  rad950 --call NQ6N --grid EN61 --band 17m --audio-in \"USB Audio\" --audio-out \"USB Audio\" --rigctld 127.0.0.1:4532 --live-tx

  # Auto-spawn Hamlib rigctld for an IC-7300 on COM3 (or /dev/ttyUSB0)
  rad950 --call NQ6N --grid EN61 --rig-model 3073 --rig-port COM3 --rig-baud 19200

  # USB-connected Radtel RT-950 Pro bridged to rad950.org Web Remote (headless TUI)
  rad950 --text --rt950-port /dev/ttyUSB0 --call NQ6N --grid EN61 --band 17m --spot-key r950_... --web-remote --web-remote-allow-tx --web-remote-name \"shack-rt950\"

  # TCI WebSocket SDR (ExpertSDR / Thetis / Hermes Lite 2)
  rad950 --tci 127.0.0.1:50001 --call NQ6N --grid EN61 --band 10m

DOCS & AI ASSISTANT GUIDE:
  https://rad950.org/SKILL.md";

#[derive(Parser, Debug)]
#[command(
    name = "rad950",
    version,
    about = "Cross-platform RAD-950 / RAD950-60 weak-signal HF 4-FSK transceiver (GUI default, --text TUI)",
    after_help = AFTER_HELP
)]
struct Cli {
    /// Run in terminal TUI mode instead of the graphical desktop UI
    #[arg(long, help_heading = "Interface & Discovery")]
    text: bool,

    /// List available audio input and output devices and exit
    #[arg(long, help_heading = "Interface & Discovery")]
    list_audio: bool,

    /// List available serial ports (for --rt950-port or --rig-port) and exit
    #[arg(long, help_heading = "Interface & Discovery")]
    list_serial: bool,

    /// Path to persistent TOML config file (defaults to platform config dir)
    #[arg(long, help_heading = "Interface & Discovery")]
    config: Option<PathBuf>,

    /// Station callsign with optional numeric suffix (e.g. NQ6N or NQ6N-1)
    #[arg(long, help_heading = "Station & Operating Mode")]
    call: Option<String>,

    /// 4-character Maidenhead grid square (e.g. EN61) or NONE
    #[arg(long, help_heading = "Station & Operating Mode")]
    grid: Option<String>,

    /// Calling band: 10m (28.180), 12m (24.920), 15m (21.140), or 17m (18.104 MHz USB)
    #[arg(long, help_heading = "Station & Operating Mode")]
    band: Option<String>,

    /// Custom USB dial frequency in Hz (overrides --band)
    #[arg(long, help_heading = "Station & Operating Mode")]
    freq: Option<u64>,

    /// Preferred audio subchannel index (0..7 = 600..2175 Hz, default 4 = 1500 Hz)
    #[arg(long, help_heading = "Station & Operating Mode")]
    subch: Option<usize>,

    /// Radio backend: audio (soundcard + Hamlib), tci, hackrf, or rt950 (USB RT-950 Pro)
    #[arg(long, value_enum, help_heading = "Station & Operating Mode")]
    backend: Option<CliBackend>,

    /// Enable Dry TX safety mode (do not key rig/TCI/HackRF/RT-950)
    #[arg(long, help_heading = "Station & Operating Mode")]
    dry_tx: bool,

    /// Disable Dry TX safety mode (allow live RF TX)
    #[arg(long, help_heading = "Station & Operating Mode")]
    live_tx: bool,

    /// Start in RAD950-60 1-minute ultra-weak-signal mode
    #[arg(long, help_heading = "Station & Operating Mode")]
    mode60: bool,

    /// Enable Hourly UTC Rendezvous schedule following & 50/50 Call & Listen
    #[arg(long, help_heading = "Station & Operating Mode")]
    rendezvous: bool,

    /// Hunter auto-answer mode: off, new-grid, new-band, or any
    #[arg(long, help_heading = "Station & Operating Mode")]
    auto_answer: Option<String>,

    /// Unattended TX watchdog timeout in minutes (0 = disabled, default 10)
    #[arg(long, help_heading = "Station & Operating Mode")]
    tx_watchdog: Option<u32>,

    /// Enable audible alert chimes on directed calls / completed QSOs
    #[arg(long, help_heading = "Station & Operating Mode")]
    alert_tones: bool,

    /// Send periodic 10-minute CW ID (DE <CALL> RAD950.ORG) on CQ slots (Audio/TCI backends)
    #[arg(long, help_heading = "Station & Operating Mode")]
    cw_id: bool,

    /// Audio input device name substring (for --backend audio)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    audio_in: Option<String>,

    /// Audio output device name substring (for --backend audio)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    audio_out: Option<String>,

    /// TX audio output amplitude (0.05..0.95)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    tx_audio_gain: Option<f32>,

    /// Hamlib rigctld TCP address (e.g. 127.0.0.1:4532)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    rigctld: Option<String>,

    /// Optional Hamlib rig model ID to auto-spawn rigctld (e.g. 3073 for IC-7300)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    rig_model: Option<u32>,

    /// Optional serial port for auto-spawned rigctld (e.g. /dev/ttyUSB0 or COM3)
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    rig_port: Option<String>,

    /// Optional serial baud rate for auto-spawned rigctld
    #[arg(long, help_heading = "Audio + Hamlib CAT/PTT")]
    rig_baud: Option<u32>,

    /// Serial port for a USB-connected Radtel RT-950 Pro running Ferrite (e.g. /dev/ttyUSB0 or COM3; implies --backend rt950)
    #[arg(long, help_heading = "Radtel RT-950 Pro (USB Serial)")]
    rt950_port: Option<String>,

    /// TCI WebSocket server address (e.g. 127.0.0.1:50001; implies --backend tci)
    #[arg(long, help_heading = "TCI WebSocket SDR")]
    tci: Option<String>,

    /// HackRF serial number suffix (for --backend hackrf)
    #[arg(long, help_heading = "HackRF One")]
    hackrf_serial: Option<String>,

    /// HackRF RX LNA gain (0..40 dB)
    #[arg(long, help_heading = "HackRF One")]
    lna: Option<u32>,

    /// HackRF RX VGA gain (0..62 dB)
    #[arg(long, help_heading = "HackRF One")]
    vga: Option<u32>,

    /// HackRF TX VGA gain (0..47 dB)
    #[arg(long, help_heading = "HackRF One")]
    tx_gain: Option<u32>,

    /// HackRF crystal calibration in ppm
    #[arg(long, help_heading = "HackRF One")]
    ppm: Option<f64>,

    /// ADIF QSO log file path
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    log: Option<PathBuf>,

    /// Band activity log file path (WSJT-X-style ALL.TXT)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    all_txt: Option<PathBuf>,

    /// Automatically upload completed QSOs to rad950.org (requires --spot-key)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    auto_upload_qso: bool,

    /// Enable live spotting to POST /api/spots (requires --spot-key)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    spot: bool,

    /// Live spotting API URL (default https://rad950.org/api/spots)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    spot_url: Option<String>,

    /// rad950.org Bearer API key (r950_..., or RAD950_SPOT_KEY env var)
    #[arg(long, env = "RAD950_SPOT_KEY", help_heading = "Logging, Spotting & Web Remote")]
    spot_key: Option<String>,

    /// Connect this station to rad950.org Web Remote Control (/on-the-air/remote; requires --spot-key)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    web_remote: bool,

    /// Allow remote TX control over Web Remote (safety: defaults to RX-only if omitted)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    web_remote_allow_tx: bool,

    /// Display name for this station in rad950.org Web Remote
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    web_remote_name: Option<String>,

    /// Stable station ID for rad950.org Web Remote (auto-generated if omitted)
    #[arg(long, help_heading = "Logging, Spotting & Web Remote")]
    web_remote_id: Option<String>,
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    if cli.list_audio {
        let (ins, outs) = list_audio_devices();
        println!("Audio Input Devices:");
        for d in ins {
            println!("  - {d}");
        }
        println!("Audio Output Devices:");
        for d in outs {
            println!("  - {d}");
        }
        return ExitCode::SUCCESS;
    }

    if cli.list_serial {
        let ports = list_serial_ports();
        println!("Serial Ports:");
        if ports.is_empty() {
            println!("  (none detected)");
        } else {
            for p in ports {
                println!("  - {p}");
            }
        }
        return ExitCode::SUCCESS;
    }

    let config_path = cli.config.unwrap_or_else(default_config_path);
    let mut saved = SavedConfig::load(&config_path);

    if let Some(v) = cli.call { saved.call = v; }
    if let Some(v) = cli.grid { saved.grid = v; }
    if let Some(v) = cli.band { saved.band = v; saved.dial_hz = None; }
    if let Some(v) = cli.freq { saved.dial_hz = Some(v); }
    if let Some(v) = cli.subch { saved.subch = v; }
    if let Some(b) = cli.backend {
        saved.backend = match b {
            CliBackend::Audio => "audio",
            CliBackend::Tci => "tci",
            CliBackend::Hackrf => "hackrf",
            CliBackend::Rt950 => "rt950",
        }
        .to_string();
    }
    if let Some(v) = cli.rt950_port {
        saved.rt950_port = v;
        if cli.backend.is_none() {
            saved.backend = "rt950".to_string();
        }
    }
    if let Some(v) = cli.audio_in { saved.audio_in = v; }
    if let Some(v) = cli.audio_out { saved.audio_out = v; }
    if let Some(v) = cli.tx_audio_gain { saved.tx_audio_gain = v; }
    if let Some(v) = cli.rigctld { saved.rigctld_addr = v; }
    if let Some(v) = cli.rig_model { saved.rig_model = Some(v); }
    if let Some(v) = cli.rig_port { saved.rig_port = v; }
    if let Some(v) = cli.rig_baud { saved.rig_baud = Some(v); }
    if let Some(v) = cli.tci {
        saved.tci_addr = v;
        if cli.backend.is_none() {
            saved.backend = "tci".to_string();
        }
    }
    if let Some(v) = cli.hackrf_serial { saved.hackrf_serial = v; }
    if let Some(v) = cli.lna { saved.lna_db = v; }
    if let Some(v) = cli.vga { saved.vga_db = v; }
    if let Some(v) = cli.tx_gain { saved.tx_vga_db = v; }
    if let Some(v) = cli.ppm { saved.ppm = v; }
    if cli.dry_tx { saved.dry_tx = true; }
    if cli.live_tx { saved.dry_tx = false; }
    if cli.mode60 { saved.mode60 = true; }
    if cli.rendezvous {
        saved.rdv_follow = true;
        saved.rdv_call_listen = true;
    }
    if let Some(v) = cli.auto_answer { saved.auto_answer = v; }
    if let Some(v) = cli.tx_watchdog { saved.tx_watchdog_mins = v; }
    if cli.alert_tones { saved.alert_tones = true; }
    if cli.cw_id { saved.cw_id = true; }
    if let Some(v) = cli.log { saved.log_path = v.to_string_lossy().into_owned(); }
    if let Some(v) = cli.all_txt { saved.all_txt_path = v.to_string_lossy().into_owned(); }
    if cli.auto_upload_qso { saved.auto_upload_qso = true; }
    if cli.spot { saved.spot_enabled = true; }
    if let Some(v) = cli.spot_url { saved.spot_url = v; }
    if let Some(v) = cli.spot_key {
        if !v.is_empty() {
            saved.spot_key = v;
        }
    }
    if cli.web_remote { saved.web_remote_enabled = true; }
    if cli.web_remote_allow_tx { saved.web_remote_allow_tx = true; }
    if let Some(v) = cli.web_remote_name {
        if !v.trim().is_empty() {
            saved.web_remote_station_name = v;
        }
    }
    if let Some(v) = cli.web_remote_id {
        if !v.trim().is_empty() {
            saved.web_remote_station_id = v;
        }
    }

    let node_cfg = match saved.to_node_config() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("config error: {e}");
            return ExitCode::from(2);
        }
    };

    let mut runner = match NodeRunner::start(node_cfg) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("failed to start RAD950 node: {e}");
            return ExitCode::from(1);
        }
    };

    let res = if cli.text {
        tui::run_tui(runner.state.clone())
    } else {
        gui::run_gui(runner.state.clone(), saved, config_path)
    };
    runner.stop();

    if let Err(e) = res {
        eprintln!("RAD950 error: {e}");
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}
