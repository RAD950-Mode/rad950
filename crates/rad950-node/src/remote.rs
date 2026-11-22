//! Web Remote Control client connecting the desktop `rad950-app` to the user's
//! `rad950.org` account via a persistent outbound WebSocket.
//!
//! Streams real-time panafall rows (`386` bytes @ `~11.7 FPS`), decoded captures,
//! QSO log, events, and transceiver state, and executes remote control commands
//! with local enforcement of `web_remote_allow_tx` + dead-man TX safety.

use std::io::ErrorKind;
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::Serialize;
use serde_json::Value;
use tungstenite::client::IntoClientRequest;
use tungstenite::http::header::{AUTHORIZATION, HeaderValue};
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{connect, Error as WsError, Message, WebSocket};
use urad950::SUBCHANNELS_HZ;

use crate::engine::{AutoAnswer, Band, OpMode};
use crate::runner::{SharedNodeState, SlotPhase};
use crate::spectrum::{RowMarker, WaterfallRow, WF_BINS};

const WF_FRAME_TYPE: u8 = 0x01;
const WF_FRAME_BYTES: usize = 2 + WF_BINS; // 386 bytes

#[derive(Serialize)]
struct RemoteHeardItem {
    idx: usize,
    utc_hms: String,
    snr_db: i8,
    audio_hz: u16,
    call: String,
    grid: String,
    message: String,
    slot: u64,
    needed: String,
    badge: String,
}

#[derive(Serialize)]
struct RemoteQsoItem {
    utc_sec: u64,
    dial_hz: u64,
    band: String,
    peer: String,
    grid: String,
    rx_snr: i8,
    tx_snr: i8,
    mode60: bool,
}

#[derive(Serialize)]
struct RemoteStateSnapshot {
    #[serde(rename = "type")]
    msg_type: &'static str,
    station_id: String,
    station_name: String,
    allow_tx: bool,
    utc_ms: u64,
    call: String,
    grid: String,
    band: String,
    dial_hz: u64,
    subch: usize,
    subch_hz: u16,
    mode60: bool,
    op_mode: &'static str,
    auto_answer: &'static str,
    rdv_follow: bool,
    rdv_call_listen: bool,
    tx_enabled: bool,
    dry_tx: bool,
    phase: &'static str,
    phase_note: String,
    peak_pct: f32,
    frames_rx: u64,
    slots_rx: u64,
    slots_tx: u64,
    spots_enabled: bool,
    spots_sent: u64,
    spots_err: u64,
    backend_label: String,
    cat_status: String,
    last_tx_summary: String,
    wd_rem_secs: Option<u64>,
    heard: Vec<RemoteHeardItem>,
    qso_log: Vec<RemoteQsoItem>,
    events: Vec<String>,
}

/// Derive `wss://<host>/api/remote/station?...` from `spot_url` (e.g. `https://rad950.org/api/spots`).
pub fn derive_remote_ws_url(
    spot_url: &str,
    station_id: &str,
    station_name: &str,
    allow_tx: bool,
    call: &str,
    grid: &str,
) -> Option<String> {
    let trimmed = spot_url.trim();
    let (scheme, rest) = if let Some(r) = trimmed.strip_prefix("https://") {
        ("wss://", r)
    } else if let Some(r) = trimmed.strip_prefix("http://") {
        ("ws://", r)
    } else if trimmed.is_empty() {
        ("wss://", "rad950.org")
    } else {
        return None;
    };
    let host_port = rest.split('/').next()?.trim();
    if host_port.is_empty() {
        return None;
    }
    Some(format!(
        "{scheme}{host_port}/api/remote/station?station_id={}&name={}&allow_tx={}&call={}&grid={}",
        url_encode(station_id),
        url_encode(station_name),
        if allow_tx { "1" } else { "0" },
        url_encode(call),
        url_encode(grid),
    ))
}

fn url_encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_' | b'.' | b'~') {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

/// Pack a `WaterfallRow` into a 386-byte binary frame: `[0x01, marker, u8[384]]`.
/// Each bin encodes `-110.0 .. +17.5 dBFS` in `0.5 dB` steps.
pub fn encode_waterfall_row(row: &WaterfallRow) -> Vec<u8> {
    let mut buf = Vec::with_capacity(WF_FRAME_BYTES);
    buf.push(WF_FRAME_TYPE);
    buf.push(match row.marker {
        RowMarker::Normal => 0,
        RowMarker::SlotBoundary => 1,
        RowMarker::Tx => 2,
    });
    for &db10 in &row.bins_db10 {
        let q = ((db10 as i32 + 1100) / 5).clamp(0, 255) as u8;
        buf.push(q);
    }
    buf
}

fn set_stream_nonblocking(ws: &mut WebSocket<MaybeTlsStream<TcpStream>>, nonblocking: bool) {
    match ws.get_mut() {
        MaybeTlsStream::Plain(s) => {
            let _ = s.set_nonblocking(nonblocking);
        }
        MaybeTlsStream::Rustls(s) => {
            let _ = s.get_mut().set_nonblocking(nonblocking);
        }
        _ => {}
    }
}

fn phase_str(p: SlotPhase) -> &'static str {
    match p {
        SlotPhase::Waiting => "waiting",
        SlotPhase::Rx => "rx",
        SlotPhase::Tx => "tx",
        SlotPhase::DryTx => "dry_tx",
    }
}

fn op_mode_key(m: OpMode) -> &'static str {
    match m {
        OpMode::Monitor => "monitor",
        OpMode::AutoCqEven => "run_even",
        OpMode::AutoCqOdd => "run_odd",
        OpMode::Pounce => "pounce",
    }
}

fn auto_answer_key(a: AutoAnswer) -> &'static str {
    match a {
        AutoAnswer::Off => "off",
        AutoAnswer::NewGrid => "new-grid",
        AutoAnswer::NewBandGrid => "new-band",
        AutoAnswer::AnyNeeded => "any",
    }
}

fn needed_key(n: crate::runner::NeededTag) -> &'static str {
    match n {
        crate::runner::NeededTag::ToMe => "to_me",
        crate::runner::NeededTag::NewGrid => "new_grid",
        crate::runner::NeededTag::NewBandGrid => "new_band_grid",
        crate::runner::NeededTag::NewCall => "new_call",
        crate::runner::NeededTag::Dupe => "dupe",
        crate::runner::NeededTag::Info => "info",
    }
}

fn build_state_snapshot(st: &SharedNodeState) -> RemoteStateSnapshot {
    let utc_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;

    let heard = st
        .heard
        .iter()
        .enumerate()
        .rev()
        .take(120)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .map(|(idx, h)| RemoteHeardItem {
            idx,
            utc_hms: h.utc_hms.clone(),
            snr_db: h.snr_db,
            audio_hz: h.audio_hz,
            call: h.call.clone(),
            grid: h.grid.clone(),
            message: h.message.clone(),
            slot: h.slot,
            needed: needed_key(h.needed).to_string(),
            badge: h.needed.badge().to_string(),
        })
        .collect();

    let qso_log = st
        .qso_log
        .iter()
        .rev()
        .take(100)
        .map(|q| RemoteQsoItem {
            utc_sec: q.utc_sec,
            dial_hz: q.dial_hz,
            band: q.band.label().to_string(),
            peer: q.peer.clone(),
            grid: q.grid.clone(),
            rx_snr: q.rx_snr,
            tx_snr: q.tx_snr,
            mode60: q.mode60,
        })
        .collect();

    let events = st.events.iter().rev().take(80).cloned().collect::<Vec<_>>().into_iter().rev().collect();

    let subch = st.engine.preferred_subch.min(SUBCHANNELS_HZ.len() - 1);

    RemoteStateSnapshot {
        msg_type: "state",
        station_id: st.web_remote_station_id.clone(),
        station_name: st.web_remote_station_name.clone(),
        allow_tx: st.web_remote_allow_tx,
        utc_ms,
        call: st.engine.my_call_disp.clone(),
        grid: st.engine.my_grid_str.clone(),
        band: st.engine.band.label().to_string(),
        dial_hz: st.engine.dial_hz,
        subch,
        subch_hz: SUBCHANNELS_HZ[subch],
        mode60: st.engine.mode60,
        op_mode: op_mode_key(st.engine.mode),
        auto_answer: auto_answer_key(st.engine.auto_answer),
        rdv_follow: st.engine.rdv_follow,
        rdv_call_listen: st.engine.rdv_call_listen,
        tx_enabled: st.tx_enabled,
        dry_tx: st.dry_tx,
        phase: phase_str(st.phase),
        phase_note: st.phase_note.clone(),
        peak_pct: st.peak_pct,
        frames_rx: st.frames_rx,
        slots_rx: st.slots_rx,
        slots_tx: st.slots_tx,
        spots_enabled: st.spot_enabled,
        spots_sent: st.spots_sent,
        spots_err: st.spots_err,
        backend_label: st.backend_label.clone(),
        cat_status: st.cat_status.clone(),
        last_tx_summary: st.last_tx_summary.clone(),
        wd_rem_secs: st.watchdog_remaining_secs(),
        heard,
        qso_log,
        events,
    }
}

/// Fingerprint of state fields that should trigger an immediate JSON state push.
#[derive(PartialEq, Eq, Default)]
struct StateFingerprint {
    frames_rx: u64,
    slots_rx: u64,
    slots_tx: u64,
    heard_len: usize,
    qso_len: usize,
    events_len: usize,
    phase: u8,
    dial_hz: u64,
    subch: usize,
    mode60: bool,
    op_mode: u8,
    auto_answer: u8,
    rdv_follow: bool,
    rdv_call_listen: bool,
    tx_enabled: bool,
    dry_tx: bool,
    allow_tx: bool,
    station_name: String,
}

fn fingerprint(st: &SharedNodeState) -> StateFingerprint {
    StateFingerprint {
        frames_rx: st.frames_rx,
        slots_rx: st.slots_rx,
        slots_tx: st.slots_tx,
        heard_len: st.heard.len(),
        qso_len: st.qso_log.len(),
        events_len: st.events.len(),
        phase: st.phase as u8,
        dial_hz: st.engine.dial_hz,
        subch: st.engine.preferred_subch,
        mode60: st.engine.mode60,
        op_mode: st.engine.mode as u8,
        auto_answer: st.engine.auto_answer as u8,
        rdv_follow: st.engine.rdv_follow,
        rdv_call_listen: st.engine.rdv_call_listen,
        tx_enabled: st.tx_enabled,
        dry_tx: st.dry_tx,
        allow_tx: st.web_remote_allow_tx,
        station_name: st.web_remote_station_name.clone(),
    }
}

/// Handle a JSON command from a web client. Returns `(ok, optional_error, armed_tx_flag)`.
fn handle_remote_command(
    state: &Arc<Mutex<SharedNodeState>>,
    msg: &Value,
    remote_armed_tx: &mut bool,
) -> (bool, Option<String>) {
    let cmd = msg.get("cmd").and_then(|v| v.as_str()).unwrap_or("");
    let mut st = state.lock().unwrap();

    let require_tx = |allow_tx: bool| -> Result<(), String> {
        if allow_tx {
            Ok(())
        } else {
            Err("Remote TX is disabled on this station (enable 'Allow remote TX' in desktop Settings).".to_string())
        }
    };

    match cmd {
        "halt_tx" => {
            st.tx_enabled = false;
            st.engine.set_mode(OpMode::Monitor);
            *remote_armed_tx = false;
            st.push_event("Web Remote: HALT TX -> Monitor".to_string());
            (true, None)
        }
        "set_band" => {
            let band_str = msg.get("band").and_then(|v| v.as_str()).unwrap_or("");
            if let Some((band, hz)) = Band::parse(band_str) {
                st.pending_dial_qsy = Some(hz);
                st.push_event(format!("Web Remote: Band -> {} ({:.3} MHz)", band.label(), hz as f64 / 1e6));
                (true, None)
            } else {
                (false, Some(format!("Unknown band: {band_str}")))
            }
        }
        "set_dial" => {
            if let Some(hz) = msg.get("dial_hz").and_then(|v| v.as_u64()) {
                if (1_800_000..=54_000_000).contains(&hz) {
                    st.pending_dial_qsy = Some(hz);
                    st.push_event(format!("Web Remote: QSY -> {:.3} MHz", hz as f64 / 1e6));
                    (true, None)
                } else {
                    (false, Some("Dial frequency out of range".to_string()))
                }
            } else {
                (false, Some("Missing dial_hz".to_string()))
            }
        }
        "set_subch" => {
            if let Some(subch) = msg.get("subch").and_then(|v| v.as_u64()) {
                let idx = (subch as usize).min(SUBCHANNELS_HZ.len() - 1);
                st.engine.preferred_subch = idx;
                (true, None)
            } else {
                (false, Some("Missing subch".to_string()))
            }
        }
        "set_mode60" => {
            if let Some(m60) = msg.get("mode60").and_then(|v| v.as_bool()) {
                st.engine.set_mode60(m60);
                st.push_event(format!(
                    "Web Remote: Mode -> {}",
                    if m60 { "RAD950-60 (1-Min)" } else { "RAD950 (10s)" }
                ));
                (true, None)
            } else {
                (false, Some("Missing mode60".to_string()))
            }
        }
        "clear_heard" => {
            st.heard.clear();
            (true, None)
        }
        "set_tx_enabled" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            if let Some(en) = msg.get("enabled").and_then(|v| v.as_bool()) {
                st.tx_enabled = en;
                *remote_armed_tx = en;
                if en {
                    st.reset_watchdog();
                }
                st.push_event(format!("Web Remote: TX {}", if en { "ENABLED" } else { "DISABLED" }));
                (true, None)
            } else {
                (false, Some("Missing enabled".to_string()))
            }
        }
        "set_dry_tx" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            if let Some(dry) = msg.get("dry_tx").and_then(|v| v.as_bool()) {
                st.dry_tx = dry;
                st.push_event(format!("Web Remote: Dry TX -> {}", if dry { "ON" } else { "OFF" }));
                (true, None)
            } else {
                (false, Some("Missing dry_tx".to_string()))
            }
        }
        "set_op_mode" => {
            let mode_str = msg.get("mode").and_then(|v| v.as_str()).unwrap_or("");
            let next_mode = match mode_str {
                "monitor" => OpMode::Monitor,
                "run_even" => OpMode::AutoCqEven,
                "run_odd" => OpMode::AutoCqOdd,
                _ => return (false, Some(format!("Invalid op mode: {mode_str}"))),
            };
            if next_mode != OpMode::Monitor {
                if let Err(e) = require_tx(st.web_remote_allow_tx) {
                    return (false, Some(e));
                }
            }
            st.engine.set_mode(next_mode);
            st.reset_watchdog();
            st.push_event(format!("Web Remote: Mode -> {}", next_mode.label()));
            (true, None)
        }
        "set_auto_answer" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            let raw = msg.get("auto_answer").and_then(|v| v.as_str()).unwrap_or("off");
            match AutoAnswer::parse(raw) {
                Ok(aa) => {
                    st.engine.auto_answer = aa;
                    st.push_event(format!("Web Remote: Hunter -> {}", aa.label()));
                    (true, None)
                }
                Err(e) => (false, Some(e)),
            }
        }
        "set_rdv_follow" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            if let Some(v) = msg.get("enabled").and_then(|v| v.as_bool()) {
                st.engine.rdv_follow = v;
                (true, None)
            } else {
                (false, Some("Missing enabled".to_string()))
            }
        }
        "set_rdv_call_listen" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            if let Some(v) = msg.get("enabled").and_then(|v| v.as_bool()) {
                st.engine.rdv_call_listen = v;
                (true, None)
            } else {
                (false, Some("Missing enabled".to_string()))
            }
        }
        "answer" => {
            if let Err(e) = require_tx(st.web_remote_allow_tx) {
                return (false, Some(e));
            }
            let idx = msg.get("idx").and_then(|v| v.as_u64()).map(|n| n as usize);
            let target = idx.and_then(|i| st.heard.get(i).cloned());
            if let Some(h) = target {
                if st.engine.pounce_peer(h.raw_call, h.grid, h.snr_db, h.slot, h.is_rx0_to_me) {
                    st.tx_enabled = true;
                    *remote_armed_tx = true;
                    st.reset_watchdog();
                    st.push_event(format!("Web Remote: Pounce queued -> {}", h.call));
                    (true, None)
                } else {
                    (false, Some(format!("Cannot pounce {}", h.call)))
                }
            } else {
                (false, Some("Decoded station entry not found".to_string()))
            }
        }
        other => (false, Some(format!("Unsupported command: {other}"))),
    }
}

/// Main background thread driving the Web Remote Control connection.
pub fn remote_loop(state: Arc<Mutex<SharedNodeState>>, running: Arc<AtomicBool>) {
    let mut backoff_ms: u64 = 1500;

    while running.load(Ordering::Relaxed) {
        let (enabled, spot_url, spot_key, station_id, station_name, allow_tx, call, grid) = {
            let st = state.lock().unwrap();
            (
                st.web_remote_enabled,
                st.spot_url.clone(),
                st.spot_key.clone(),
                st.web_remote_station_id.clone(),
                st.web_remote_station_name.clone(),
                st.web_remote_allow_tx,
                st.engine.my_call_disp.clone(),
                st.engine.my_grid_str.clone(),
            )
        };

        if !enabled {
            {
                let mut st = state.lock().unwrap();
                st.web_remote_connected = false;
                st.web_remote_status = "Off".to_string();
            }
            backoff_ms = 1500;
            thread::sleep(Duration::from_millis(250));
            continue;
        }

        if spot_key.trim().is_empty() {
            {
                let mut st = state.lock().unwrap();
                st.web_remote_connected = false;
                st.web_remote_status = "API key required (set rad950.org API key)".to_string();
            }
            thread::sleep(Duration::from_millis(500));
            continue;
        }

        let Some(ws_url) = derive_remote_ws_url(
            &spot_url,
            &station_id,
            &station_name,
            allow_tx,
            &call,
            &grid,
        ) else {
            {
                let mut st = state.lock().unwrap();
                st.web_remote_connected = false;
                st.web_remote_status = "Invalid rad950.org URL".to_string();
            }
            thread::sleep(Duration::from_millis(1000));
            continue;
        };

        {
            let mut st = state.lock().unwrap();
            st.web_remote_connected = false;
            st.web_remote_status = "Connecting...".to_string();
        }

        let req = match ws_url.into_client_request() {
            Ok(mut r) => {
                if let Ok(val) = HeaderValue::from_str(&format!("Bearer {}", spot_key.trim())) {
                    r.headers_mut().insert(AUTHORIZATION, val);
                }
                r
            }
            Err(e) => {
                let mut st = state.lock().unwrap();
                st.web_remote_status = format!("URL error: {e}");
                thread::sleep(Duration::from_millis(2000));
                continue;
            }
        };

        let mut ws = match connect(req) {
            Ok((socket, _resp)) => socket,
            Err(e) => {
                {
                    let mut st = state.lock().unwrap();
                    st.web_remote_connected = false;
                    st.web_remote_status = format!("Disconnected ({e})");
                }
                let steps = (backoff_ms / 100).max(1);
                for _ in 0..steps {
                    if !running.load(Ordering::Relaxed) || !state.lock().unwrap().web_remote_enabled {
                        break;
                    }
                    thread::sleep(Duration::from_millis(100));
                }
                backoff_ms = (backoff_ms * 2).min(15_000);
                continue;
            }
        };

        backoff_ms = 1500;
        set_stream_nonblocking(&mut ws, true);

        {
            let mut st = state.lock().unwrap();
            st.web_remote_connected = true;
            st.web_remote_status = format!("Connected as \"{}\"", station_name);
            st.push_event(format!("Web Remote connected to rad950.org as \"{station_name}\""));
        }

        // Send initial state snapshot immediately
        let mut last_wf_seq = {
            let st = state.lock().unwrap();
            let snap = build_state_snapshot(&st);
            if let Ok(txt) = serde_json::to_string(&snap) {
                let _ = ws.send(Message::Text(txt.into()));
            }
            st.wf_seq
        };
        let mut last_fp = fingerprint(&state.lock().unwrap());
        let mut last_state_push = Instant::now();
        let mut last_ping = Instant::now();
        let mut remote_armed_tx = false;

        'connected: while running.load(Ordering::Relaxed) {
            // Check if settings changed requiring disconnect/reconnect
            {
                let st = state.lock().unwrap();
                if !st.web_remote_enabled
                    || st.spot_url != spot_url
                    || st.spot_key != spot_key
                    || st.web_remote_station_id != station_id
                {
                    let _ = ws.close(None);
                    break 'connected;
                }
            }

            // 1. Drain incoming messages (non-blocking)
            loop {
                match ws.read() {
                    Ok(Message::Text(txt)) => {
                        if let Ok(val) = serde_json::from_str::<Value>(txt.as_str()) {
                            let mtype = val.get("type").and_then(|v| v.as_str()).unwrap_or("");
                            if mtype == "cmd" {
                                let cmd_name = val.get("cmd").and_then(|v| v.as_str()).unwrap_or("").to_string();
                                let (ok, err) = handle_remote_command(&state, &val, &mut remote_armed_tx);
                                let ack = serde_json::json!({
                                    "type": "ack",
                                    "cmd": cmd_name,
                                    "ok": ok,
                                    "error": err,
                                });
                                if ws.send(Message::Text(ack.to_string().into())).is_err() {
                                    break 'connected;
                                }
                            }
                        }
                    }
                    Ok(Message::Ping(payload)) => {
                        if ws.send(Message::Pong(payload)).is_err() {
                            break 'connected;
                        }
                    }
                    Ok(Message::Close(_)) => {
                        break 'connected;
                    }
                    Ok(_) => {}
                    Err(WsError::Io(ref io_err))
                        if io_err.kind() == ErrorKind::WouldBlock
                            || io_err.kind() == ErrorKind::TimedOut =>
                    {
                        break;
                    }
                    Err(_) => {
                        break 'connected;
                    }
                }
            }

            // 2. Stream any new waterfall rows (oldest-first)
            let new_rows: Vec<Vec<u8>> = {
                let st = state.lock().unwrap();
                let delta = st.wf_seq.wrapping_sub(last_wf_seq) as usize;
                if delta > 0 {
                    let count = delta.min(st.waterfall.len()).min(24);
                    let mut rows = Vec::with_capacity(count);
                    // st.waterfall[0] is newest, so iterate from count-1 down to 0
                    for i in (0..count).rev() {
                        rows.push(encode_waterfall_row(&st.waterfall[i]));
                    }
                    last_wf_seq = st.wf_seq;
                    rows
                } else {
                    Vec::new()
                }
            };
            for frame in new_rows {
                if ws.send(Message::Binary(frame.into())).is_err() {
                    break 'connected;
                }
            }

            // 3. Push state snapshot on change or 1 Hz heartbeat
            let maybe_snap = {
                let st = state.lock().unwrap();
                let fp = fingerprint(&st);
                if fp != last_fp || last_state_push.elapsed() >= Duration::from_secs(1) {
                    last_fp = fp;
                    last_state_push = Instant::now();
                    serde_json::to_string(&build_state_snapshot(&st)).ok()
                } else {
                    None
                }
            };
            if let Some(txt) = maybe_snap {
                if ws.send(Message::Text(txt.into())).is_err() {
                    break 'connected;
                }
            }

            // 4. Periodic keepalive ping
            if last_ping.elapsed() >= Duration::from_secs(25) {
                last_ping = Instant::now();
                if ws.send(Message::Text(r#"{"type":"ping"}"#.into())).is_err() {
                    break 'connected;
                }
            }

            thread::sleep(Duration::from_millis(25));
        }

        // Dead-man safety: if remote client had armed TX and connection dropped, disarm TX
        {
            let mut st = state.lock().unwrap();
            st.web_remote_connected = false;
            if remote_armed_tx && st.tx_enabled {
                st.tx_enabled = false;
                st.push_event("Web Remote disconnected while TX armed — TX auto-disabled for safety".to_string());
            }
            if st.web_remote_enabled {
                st.web_remote_status = "Reconnecting...".to_string();
            } else {
                st.web_remote_status = "Off".to_string();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derive_ws_url_handles_https_and_http() {
        let u = derive_remote_ws_url(
            "https://rad950.org/api/spots",
            "abc123",
            "dipole-4821",
            true,
            "NQ6N",
            "EN61",
        )
        .unwrap();
        assert_eq!(
            u,
            "wss://rad950.org/api/remote/station?station_id=abc123&name=dipole-4821&allow_tx=1&call=NQ6N&grid=EN61"
        );

        let local = derive_remote_ws_url(
            "http://127.0.0.1:8787/api/spots",
            "id1",
            "My Shack",
            false,
            "NQ6N",
            "EN61",
        )
        .unwrap();
        assert_eq!(
            local,
            "ws://127.0.0.1:8787/api/remote/station?station_id=id1&name=My%20Shack&allow_tx=0&call=NQ6N&grid=EN61"
        );
    }

    #[test]
    fn waterfall_encoding_roundtrip_range() {
        let mut row = WaterfallRow {
            bins_db10: [-950; WF_BINS],
            marker: RowMarker::SlotBoundary,
        };
        row.bins_db10[0] = -1100; // -110.0 dBFS -> 0
        row.bins_db10[1] = -600; // -60.0 dBFS -> 100
        row.bins_db10[2] = 0; // 0.0 dBFS -> 220
        let enc = encode_waterfall_row(&row);
        assert_eq!(enc.len(), 386);
        assert_eq!(enc[0], 0x01);
        assert_eq!(enc[1], 1);
        assert_eq!(enc[2], 0);
        assert_eq!(enc[3], 100);
        assert_eq!(enc[4], 220);
    }
}
