mod gui;
mod tui;
mod waterfall;

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, ValueEnum};
use rad950_node::audio::list_audio_devices;
use rad950_node::config::{default_config_path, SavedConfig};
use rad950_node::runner::NodeRunner;

#[derive(Clone, Copy, Debug, ValueEnum)]
enum CliBackend {
    Audio,
    Tci,
    Hackrf,
    Rt950,
}

#[derive(Parser, Debug)]
#[command(name = "rad950", about = "Cross-platform RAD950 / RAD950-60 transceiver (GUI default, --text TUI)")]
struct Cli {
    /// Run in terminal TUI mode instead of the graphical desktop UI
    #[arg(long)]
    text: bool,

    /// List available audio input and output devices and exit
    #[arg(long)]
    list_audio: bool,

    /// Path to persistent TOML config file (defaults to platform config dir)
    #[arg(long)]
    config: Option<PathBuf>,

    /// Station callsign with optional numeric suffix (e.g. NQ6N or NQ6N-1)
    #[arg(long)]
    call: Option<String>,

    /// 4-character Maidenhead grid square (e.g. EN61) or NONE
    #[arg(long)]
    grid: Option<String>,

    /// Calling band: 10m, 12m, 15m, or 17m
    #[arg(long)]
    band: Option<String>,

    /// Custom USB dial frequency in Hz (overrides --band)
    #[arg(long)]
    freq: Option<u64>,

    /// Preferred audio subchannel index (0..7 = 600..2175 Hz)
    #[arg(long)]
    subch: Option<usize>,

    /// Radio backend: audio (soundcard + Hamlib), tci (WebSocket CAT+audio), hackrf, or rt950 (serial RT-950 Pro)
    #[arg(long, value_enum)]
    backend: Option<CliBackend>,

    /// Serial port for a USB-connected Radtel RT-950 Pro running Ferrite (e.g. /dev/ttyUSB0; implies --backend rt950)
    #[arg(long)]
    rt950_port: Option<String>,

    /// Audio input device name substring (for --backend audio)
    #[arg(long)]
    audio_in: Option<String>,

    /// Audio output device name substring (for --backend audio)
    #[arg(long)]
    audio_out: Option<String>,

    /// TX audio output amplitude (0.05..0.95)
    #[arg(long)]
    tx_audio_gain: Option<f32>,

    /// Hamlib rigctld TCP address (e.g. 127.0.0.1:4532)
    #[arg(long)]
    rigctld: Option<String>,

    /// Optional Hamlib rig model ID to auto-spawn rigctld (e.g. 3073 for IC-7300)
    #[arg(long)]
    rig_model: Option<u32>,

    /// Optional serial port for auto-spawned rigctld (e.g. /dev/ttyUSB0 or COM3)
    #[arg(long)]
    rig_port: Option<String>,

    /// Optional serial baud rate for auto-spawned rigctld
    #[arg(long)]
    rig_baud: Option<u32>,

    /// TCI WebSocket server address (e.g. 127.0.0.1:50001)
    #[arg(long)]
    tci: Option<String>,

    /// HackRF serial number suffix (for --backend hackrf)
    #[arg(long)]
    hackrf_serial: Option<String>,

    /// HackRF RX LNA gain (0..40 dB)
    #[arg(long)]
    lna: Option<u32>,

    /// HackRF RX VGA gain (0..62 dB)
    #[arg(long)]
    vga: Option<u32>,

    /// HackRF TX VGA gain (0..47 dB)
    #[arg(long)]
    tx_gain: Option<u32>,

    /// HackRF crystal calibration in ppm
    #[arg(long)]
    ppm: Option<f64>,

    /// Enable Dry TX safety mode (do not key rig/TCI/HackRF)
    #[arg(long)]
    dry_tx: bool,

    /// Disable Dry TX safety mode (allow live RF TX)
    #[arg(long)]
    live_tx: bool,

    /// Start in RAD950-60 1-minute weak-signal mode
    #[arg(long)]
    mode60: bool,

    /// Enable Hourly UTC Rendezvous schedule following & 50/50 Call & Listen
    #[arg(long)]
    rendezvous: bool,

    /// Hunter auto-answer mode: off, new-grid, new-band, or any
    #[arg(long)]
    auto_answer: Option<String>,

    /// ADIF QSO log file path
    #[arg(long)]
    log: Option<PathBuf>,

    /// Band activity log file path (WSJT-X-style ALL.TXT)
    #[arg(long)]
    all_txt: Option<PathBuf>,

    /// Enable live spotting to POST /api/spots
    #[arg(long)]
    spot: bool,

    /// Live spotting API URL
    #[arg(long)]
    spot_url: Option<String>,

    /// Live spotting Bearer API key (or RAD950_SPOT_KEY env var)
    #[arg(long, env = "RAD950_SPOT_KEY")]
    spot_key: Option<String>,
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
    if let Some(v) = cli.log { saved.log_path = v.to_string_lossy().into_owned(); }
    if let Some(v) = cli.all_txt { saved.all_txt_path = v.to_string_lossy().into_owned(); }
    if cli.spot { saved.spot_enabled = true; }
    if let Some(v) = cli.spot_url { saved.spot_url = v; }
    if let Some(v) = cli.spot_key {
        if !v.is_empty() {
            saved.spot_key = v;
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
