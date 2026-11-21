mod gui;
mod tui;

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, ValueEnum};
use rad950_node::audio::{list_audio_devices, AudioConfig};
use rad950_node::engine::{AutoAnswer, Band};
use rad950_node::hackrf::HackrfConfig;
use rad950_node::hamlib::HamlibConfig;
use rad950_node::runner::{BackendKind, NodeConfig, NodeRunner};

#[derive(Clone, Copy, Debug, ValueEnum)]
enum CliBackend {
    Audio,
    Hackrf,
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

    /// Station callsign with optional numeric suffix (e.g. NQ6N or NQ6N-1)
    #[arg(long, default_value = "N0CALL")]
    call: String,

    /// 4-character Maidenhead grid square (e.g. EN61) or NONE
    #[arg(long, default_value = "----")]
    grid: String,

    /// Calling band: 10m, 12m, 15m, or 17m
    #[arg(long, default_value = "17m")]
    band: String,

    /// Custom USB dial frequency in Hz (overrides --band)
    #[arg(long)]
    freq: Option<u64>,

    /// Preferred audio subchannel index (0..7 = 600..2175 Hz)
    #[arg(long, default_value_t = 4)]
    subch: usize,

    /// Radio backend: audio (soundcard + Hamlib CAT) or hackrf (direct HackRF One)
    #[arg(long, value_enum, default_value_t = CliBackend::Audio)]
    backend: CliBackend,

    /// Audio input device name substring (for --backend audio)
    #[arg(long)]
    audio_in: Option<String>,

    /// Audio output device name substring (for --backend audio)
    #[arg(long)]
    audio_out: Option<String>,

    /// TX audio output amplitude (0.05..0.95)
    #[arg(long, default_value_t = 0.5)]
    tx_audio_gain: f32,

    /// Hamlib rigctld TCP address (e.g. 127.0.0.1:4532)
    #[arg(long, default_value = "")]
    rigctld: String,

    /// Optional Hamlib rig model ID to auto-spawn rigctld (e.g. 3073 for IC-7300)
    #[arg(long)]
    rig_model: Option<u32>,

    /// Optional serial port for auto-spawned rigctld (e.g. /dev/ttyUSB0 or COM3)
    #[arg(long)]
    rig_port: Option<String>,

    /// Optional serial baud rate for auto-spawned rigctld
    #[arg(long)]
    rig_baud: Option<u32>,

    /// HackRF serial number suffix (for --backend hackrf)
    #[arg(long)]
    hackrf_serial: Option<String>,

    /// HackRF RX LNA gain (0..40 dB)
    #[arg(long, default_value_t = 24)]
    lna: u32,

    /// HackRF RX VGA gain (0..62 dB)
    #[arg(long, default_value_t = 20)]
    vga: u32,

    /// HackRF TX VGA gain (0..47 dB)
    #[arg(long, default_value_t = 20)]
    tx_gain: u32,

    /// HackRF crystal calibration in ppm
    #[arg(long, default_value_t = 0.0)]
    ppm: f64,

    /// Dry TX safety mode (do not key rig/HackRF)
    #[arg(long)]
    dry_tx: bool,

    /// Start in RAD950-60 1-minute weak-signal mode
    #[arg(long)]
    mode60: bool,

    /// Enable Hourly UTC Rendezvous schedule following & 50/50 Call & Listen
    #[arg(long)]
    rendezvous: bool,

    /// Hunter auto-answer mode: off, new-grid, new-band, or any
    #[arg(long, default_value = "off")]
    auto_answer: String,

    /// ADIF QSO log file path
    #[arg(long, default_value = "rad950_log.adi")]
    log: PathBuf,

    /// Enable live spotting to POST /api/spots
    #[arg(long)]
    spot: bool,

    /// Live spotting API URL
    #[arg(long, default_value = "https://rad950.com/api/spots")]
    spot_url: String,

    /// Live spotting Bearer API key (or RAD950_SPOT_KEY env var)
    #[arg(long, env = "RAD950_SPOT_KEY", default_value = "")]
    spot_key: String,
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

    let dial_hz = match cli.freq {
        Some(f) => f,
        None => Band::parse(&cli.band)
            .map(|(_, hz)| hz)
            .unwrap_or(18_104_000),
    };
    let auto_answer = match AutoAnswer::parse(&cli.auto_answer) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };

    let backend = match cli.backend {
        CliBackend::Audio => {
            let addr = if cli.rigctld.is_empty() && cli.rig_model.is_some() {
                "127.0.0.1:4532".to_string()
            } else {
                cli.rigctld.clone()
            };
            BackendKind::Audio(AudioConfig {
                input_device: cli.audio_in,
                output_device: cli.audio_out,
                tx_gain: cli.tx_audio_gain,
                dry_tx: cli.dry_tx,
                hamlib: HamlibConfig {
                    addr,
                    spawn_model: cli.rig_model,
                    spawn_port: cli.rig_port,
                    spawn_baud: cli.rig_baud,
                },
            })
        }
        CliBackend::Hackrf => BackendKind::Hackrf(HackrfConfig {
            serial: cli.hackrf_serial,
            lna_db: cli.lna,
            vga_db: cli.vga,
            tx_vga_db: cli.tx_gain,
            ppm: cli.ppm,
            dry_tx: cli.dry_tx,
        }),
    };

    let cfg = NodeConfig {
        call: cli.call,
        grid: cli.grid,
        dial_hz,
        subch: cli.subch,
        mode60: cli.mode60,
        auto_answer,
        rdv_follow: cli.rendezvous,
        rdv_call_listen: cli.rendezvous,
        log_path: cli.log,
        spot_enabled: cli.spot,
        spot_url: cli.spot_url,
        spot_key: cli.spot_key,
        backend,
    };

    let mut runner = match NodeRunner::start(cfg) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("failed to start RAD950 node: {e}");
            return ExitCode::from(1);
        }
    };

    let res = if cli.text {
        tui::run_tui(runner.state.clone())
    } else {
        gui::run_gui(runner.state.clone())
    };
    runner.stop();

    if let Err(e) = res {
        eprintln!("RAD950 error: {e}");
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}
