//! Built-in RAD-950 AI Help Assistant client for the desktop GUI.
//!
//! - Auto-detects the computer's OS language/locale (macOS, Windows, Linux).
//! - Sends a redacted snapshot of the user's current settings, detected audio
//!   devices, serial ports, and live status to `https://rad950.org/api/assistant`.
//! - Receives plain-language answers and optional structured `proposed_settings`
//!   updates that the user can confirm with one click ("Apply Settings").

use std::sync::mpsc::{self, Receiver};
use std::sync::OnceLock;
use std::time::Duration;

use rad950_node::config::SavedConfig;
use rad950_node::runner::SharedNodeState;
use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};

const ASSISTANT_URL: &str = "https://rad950.org/api/assistant";

static SYSTEM_LOCALE: OnceLock<String> = OnceLock::new();

/// Detect the user's primary OS locale (e.g. "en-US", "de-DE", "ja-JP").
pub fn detect_system_locale() -> &'static str {
    SYSTEM_LOCALE.get_or_init(|| {
        // 1. Standard POSIX locale env vars (Linux, macOS terminal, custom overrides)
        for key in ["LC_ALL", "LC_MESSAGES", "LANG", "LANGUAGE"] {
            if let Ok(val) = std::env::var(key) {
                let norm = normalize_locale(&val);
                if !norm.is_empty() && norm != "C" && norm != "POSIX" {
                    return norm;
                }
            }
        }

        // 2. macOS GUI apps launched from Finder often have empty LANG; read AppleLocale
        #[cfg(target_os = "macos")]
        {
            if let Ok(out) = std::process::Command::new("defaults")
                .args(["read", "-g", "AppleLocale"])
                .output()
            {
                if out.status.success() {
                    let s = String::from_utf8_lossy(&out.stdout);
                    let norm = normalize_locale(&s);
                    if !norm.is_empty() {
                        return norm;
                    }
                }
            }
        }

        // 3. Windows GUI apps: query user culture via PowerShell (cached once)
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            const CREATE_NO_WINDOW: u32 = 0x0800_0000;
            if let Ok(out) = std::process::Command::new("powershell")
                .args(["-NoProfile", "-Command", "(Get-Culture).Name"])
                .creation_flags(CREATE_NO_WINDOW)
                .output()
            {
                if out.status.success() {
                    let s = String::from_utf8_lossy(&out.stdout);
                    let norm = normalize_locale(&s);
                    if !norm.is_empty() {
                        return norm;
                    }
                }
            }
        }

        "en-US".to_string()
    })
}

fn normalize_locale(raw: &str) -> String {
    let first = raw.trim().split(':').next().unwrap_or("");
    let no_enc = first.split('.').next().unwrap_or("");
    let no_mod = no_enc.split('@').next().unwrap_or("");
    no_mod.replace('_', "-").trim().to_string()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProposalState {
    Pending,
    Applied,
    Dismissed,
}

#[derive(Clone, Debug)]
pub struct ProposedSettings {
    pub summary: String,
    pub changes: Map<String, Value>,
    pub state: ProposalState,
}

#[derive(Clone, Debug)]
pub struct AssistantMessage {
    pub role: String, // "user" | "assistant" | "error"
    pub content: String,
    pub proposal: Option<ProposedSettings>,
}

#[derive(Serialize)]
struct WireMessage<'a> {
    role: &'a str,
    content: &'a str,
}

#[derive(Deserialize)]
struct ApiResponse {
    reply: Option<String>,
    error: Option<String>,
    proposed_settings: Option<WireProposal>,
}

#[derive(Deserialize)]
struct WireProposal {
    summary: String,
    changes: Map<String, Value>,
}

pub struct AssistantPanel {
    pub messages: Vec<AssistantMessage>,
    pub input: String,
    pub locale: String,
    inflight: Option<Receiver<Result<AssistantMessage, String>>>,
}

impl AssistantPanel {
    pub fn new() -> Self {
        Self {
            messages: Vec::new(),
            input: String::new(),
            locale: detect_system_locale().to_string(),
            inflight: None,
        }
    }

    pub fn is_busy(&self) -> bool {
        self.inflight.is_some()
    }

    pub fn poll(&mut self) {
        if let Some(rx) = &self.inflight {
            match rx.try_recv() {
                Ok(Ok(msg)) => {
                    self.messages.push(msg);
                    self.inflight = None;
                }
                Ok(Err(err)) => {
                    self.messages.push(AssistantMessage {
                        role: "error".to_string(),
                        content: err,
                        proposal: None,
                    });
                    self.inflight = None;
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.inflight = None;
                }
            }
        }
    }

    pub fn send_user_message(
        &mut self,
        text: String,
        cfg: &SavedConfig,
        audio_inputs: &[String],
        audio_outputs: &[String],
        serial_ports: &[String],
        st: &SharedNodeState,
    ) {
        let trimmed = text.trim().to_string();
        if trimmed.is_empty() || self.inflight.is_some() {
            return;
        }
        self.messages.push(AssistantMessage {
            role: "user".to_string(),
            content: trimmed,
            proposal: None,
        });

        // Build redacted desktop context snapshot
        let mut cfg_val = serde_json::to_value(cfg).unwrap_or_else(|_| json!({}));
        if let Some(obj) = cfg_val.as_object_mut() {
            let has_key = !cfg.spot_key.trim().is_empty();
            obj.insert(
                "spot_key".to_string(),
                json!(if has_key { "<configured>" } else { "" }),
            );
        }

        let recent_events: Vec<String> = st.events.iter().rev().take(8).cloned().collect();
        let ntp = rad950_node::ntp::ntp_status()
            .map(|(off, rtt)| json!({ "synced": true, "offset_ms": off, "rtt_ms": rtt }))
            .unwrap_or_else(|| json!({ "synced": false }));

        let desktop_context = json!({
            "os": std::env::consts::OS,
            "arch": std::env::consts::ARCH,
            "version": env!("CARGO_PKG_VERSION"),
            "settings": cfg_val,
            "audio_inputs": audio_inputs,
            "audio_outputs": audio_outputs,
            "serial_ports": serial_ports,
            "status": {
                "band": st.engine.band.label(),
                "dial_hz": st.engine.dial_hz,
                "backend_label": st.backend_label,
                "cat_status": st.cat_status,
                "tx_enabled": st.tx_enabled,
                "dry_tx": st.dry_tx,
                "slots_rx": st.slots_rx,
                "frames_rx": st.frames_rx,
                "ntp": ntp,
                "recent_events": recent_events,
            }
        });

        let wire_msgs: Vec<Value> = self
            .messages
            .iter()
            .filter(|m| m.role == "user" || m.role == "assistant")
            .rev()
            .take(16)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .map(|m| json!(WireMessage { role: &m.role, content: &m.content }))
            .collect();

        let body = json!({
            "messages": wire_msgs,
            "locale": self.locale,
            "client": "desktop",
            "desktop_context": desktop_context,
        });

        let (tx, rx) = mpsc::channel();
        self.inflight = Some(rx);

        std::thread::spawn(move || {
            let res = call_assistant_api(body);
            let _ = tx.send(res);
        });
    }
}

fn call_assistant_api(body: Value) -> Result<AssistantMessage, String> {
    let agent = ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(10))
        .timeout_read(Duration::from_secs(45))
        .build();

    let resp = agent
        .post(ASSISTANT_URL)
        .set("Content-Type", "application/json")
        .set("User-Agent", concat!("rad950-desktop/", env!("CARGO_PKG_VERSION")))
        .send_json(body);

    match resp {
        Ok(r) => {
            let parsed: ApiResponse = r
                .into_json()
                .map_err(|e| format!("Invalid assistant response: {e}"))?;
            if let Some(err) = parsed.error {
                return Err(err);
            }
            let reply = parsed
                .reply
                .unwrap_or_else(|| "Done!".to_string());
            let proposal = parsed.proposed_settings.and_then(|p| {
                if p.changes.is_empty() {
                    None
                } else {
                    Some(ProposedSettings {
                        summary: p.summary,
                        changes: p.changes,
                        state: ProposalState::Pending,
                    })
                }
            });
            Ok(AssistantMessage {
                role: "assistant".to_string(),
                content: reply,
                proposal,
            })
        }
        Err(ureq::Error::Status(code, r)) => {
            let msg = r
                .into_json::<ApiResponse>()
                .ok()
                .and_then(|p| p.error)
                .unwrap_or_else(|| format!("Assistant server returned HTTP {code}"));
            Err(msg)
        }
        Err(e) => Err(format!("Could not reach rad950.org assistant: {e}")),
    }
}

/// Apply a partial settings JSON map onto `SavedConfig` and return friendly human-readable
/// descriptions of each updated item.
pub fn describe_changes(changes: &Map<String, Value>) -> Vec<String> {
    let mut lines = Vec::new();
    for (k, v) in changes {
        let label = match k.as_str() {
            "call" => "Callsign",
            "grid" => "Grid Square",
            "band" => "Band",
            "subch" => "Audio Subchannel",
            "mode60" => "RAD950-60 (1-min mode)",
            "auto_answer" => "Auto-Answer / Auto-Pounce",
            "rdv_follow" => "Rendezvous Schedule",
            "rdv_call_listen" => "Rendezvous 50/50 Call & Listen",
            "dry_tx" => "Dry TX Safety Mode",
            "tx_watchdog_mins" => "TX Watchdog (minutes)",
            "alert_tones" => "Alert Chimes",
            "cw_id" => "10-min CW ID",
            "backend" => "Radio Backend",
            "audio_in" => "Audio Input",
            "audio_out" => "Audio Output",
            "tx_audio_gain" => "TX Audio Gain",
            "rigctld_addr" => "Hamlib rigctld Address",
            "rig_model" => "Hamlib Rig Model",
            "rig_port" => "CAT Serial Port",
            "rig_baud" => "CAT Baud Rate",
            "tci_addr" => "TCI Address",
            "rt950_port" => "RT-950 Pro USB Port",
            "auto_upload_qso" => "Auto-upload QSOs",
            "spot_enabled" => "Live Spotting",
            "web_remote_enabled" => "Web Remote Control",
            "web_remote_allow_tx" => "Web Remote TX",
            "web_remote_station_name" => "Station Name",
            _ => continue,
        };
        let val_str = match v {
            Value::Bool(true) => "On".to_string(),
            Value::Bool(false) => "Off".to_string(),
            Value::String(s) => s.clone(),
            Value::Null => "None".to_string(),
            other => other.to_string(),
        };
        lines.push(format!("{label}: {val_str}"));
    }
    lines
}

/// Apply the assistant's proposed settings onto `draft` (and string mirrors for rig model/baud).
pub fn apply_changes_to_draft(
    changes: &Map<String, Value>,
    draft: &mut SavedConfig,
    rig_model_str: &mut String,
    rig_baud_str: &mut String,
) {
    for (k, v) in changes {
        match k.as_str() {
            "call" => if let Some(s) = v.as_str() { draft.call = s.trim().to_ascii_uppercase(); },
            "grid" => if let Some(s) = v.as_str() { draft.grid = s.trim().to_ascii_uppercase(); },
            "band" => if let Some(s) = v.as_str() { draft.band = s.trim().to_string(); draft.dial_hz = None; },
            "subch" => if let Some(n) = v.as_u64() { draft.subch = (n as usize).min(7); },
            "mode60" => if let Some(b) = v.as_bool() { draft.mode60 = b; },
            "auto_answer" => if let Some(s) = v.as_str() { draft.auto_answer = s.trim().to_string(); },
            "rdv_follow" => if let Some(b) = v.as_bool() { draft.rdv_follow = b; },
            "rdv_call_listen" => if let Some(b) = v.as_bool() { draft.rdv_call_listen = b; },
            "dry_tx" => if let Some(b) = v.as_bool() { draft.dry_tx = b; },
            "tx_watchdog_mins" => if let Some(n) = v.as_u64() { draft.tx_watchdog_mins = n.min(60) as u32; },
            "alert_tones" => if let Some(b) = v.as_bool() { draft.alert_tones = b; },
            "cw_id" => if let Some(b) = v.as_bool() { draft.cw_id = b; },
            "backend" => if let Some(s) = v.as_str() { draft.backend = s.trim().to_ascii_lowercase(); },
            "audio_in" => if let Some(s) = v.as_str() { draft.audio_in = s.to_string(); },
            "audio_out" => if let Some(s) = v.as_str() { draft.audio_out = s.to_string(); },
            "tx_audio_gain" => if let Some(f) = v.as_f64() { draft.tx_audio_gain = (f as f32).clamp(0.05, 0.95); },
            "rigctld_addr" => if let Some(s) = v.as_str() { draft.rigctld_addr = s.trim().to_string(); },
            "rig_model" => {
                draft.rig_model = v.as_u64().map(|n| n as u32);
                *rig_model_str = draft.rig_model.map(|n| n.to_string()).unwrap_or_default();
            }
            "rig_port" => if let Some(s) = v.as_str() { draft.rig_port = s.trim().to_string(); },
            "rig_baud" => {
                draft.rig_baud = v.as_u64().map(|n| n as u32);
                *rig_baud_str = draft.rig_baud.map(|n| n.to_string()).unwrap_or_default();
            }
            "tci_addr" => if let Some(s) = v.as_str() { draft.tci_addr = s.trim().to_string(); },
            "rt950_port" => if let Some(s) = v.as_str() { draft.rt950_port = s.trim().to_string(); },
            "auto_upload_qso" => if let Some(b) = v.as_bool() { draft.auto_upload_qso = b; },
            "spot_enabled" => if let Some(b) = v.as_bool() { draft.spot_enabled = b; },
            "web_remote_enabled" => if let Some(b) = v.as_bool() { draft.web_remote_enabled = b; },
            "web_remote_allow_tx" => if let Some(b) = v.as_bool() { draft.web_remote_allow_tx = b; },
            "web_remote_station_name" => if let Some(s) = v.as_str() { draft.web_remote_station_name = s.trim().to_string(); },
            _ => {}
        }
    }
}
