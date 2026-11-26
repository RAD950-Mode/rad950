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

/// Render lightweight Markdown (headings, bullet/numbered lists, fenced code blocks,
/// `**bold**`, `*italic*`, `` `code` ``, and `[label](url)`) cleanly inside an `egui::Ui`.
pub fn render_markdown(ui: &mut egui::Ui, md: &str) {
    let mut in_code = false;
    let mut code_lines: Vec<&str> = Vec::new();

    let flush_code = |ui: &mut egui::Ui, lines: &mut Vec<&str>| {
        if lines.is_empty() {
            return;
        }
        let joined = lines.join("\n");
        lines.clear();
        egui::Frame::NONE
            .fill(egui::Color32::from_rgb(10, 16, 26))
            .corner_radius(5.0)
            .inner_margin(egui::Margin::same(8))
            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(35, 50, 74)))
            .show(ui, |ui| {
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(joined)
                            .monospace()
                            .size(12.5)
                            .color(egui::Color32::from_rgb(185, 222, 255)),
                    )
                    .wrap(),
                );
            });
        ui.add_space(4.0);
    };

    let normalized = md.replace("\r\n", "\n");
    for raw_line in normalized.lines() {
        let trimmed = raw_line.trim();
        if trimmed.starts_with("```") {
            if in_code {
                flush_code(ui, &mut code_lines);
                in_code = false;
            } else {
                in_code = true;
            }
            continue;
        }
        if in_code {
            code_lines.push(raw_line);
            continue;
        }
        if trimmed.is_empty() {
            ui.add_space(4.0);
            continue;
        }

        // Headings (# .. ####)
        if let Some(rest) = strip_heading(trimmed) {
            ui.add_space(3.0);
            let mut job = egui::text::LayoutJob::default();
            append_inline_md(
                &mut job,
                rest,
                14.5,
                egui::Color32::from_rgb(150, 208, 255),
                true,
            );
            ui.add(egui::Label::new(job).wrap());
            ui.add_space(2.0);
            continue;
        }

        // Bullet list (- or *)
        if let Some(rest) = trimmed
            .strip_prefix("- ")
            .or_else(|| trimmed.strip_prefix("* "))
            .or_else(|| trimmed.strip_prefix("• "))
        {
            ui.horizontal_top(|ui| {
                ui.add_space(6.0);
                ui.label(
                    egui::RichText::new("•")
                        .strong()
                        .color(egui::Color32::from_rgb(110, 185, 255)),
                );
                let mut job = egui::text::LayoutJob::default();
                append_inline_md(
                    &mut job,
                    rest,
                    13.5,
                    egui::Color32::from_rgb(230, 236, 246),
                    false,
                );
                ui.add(egui::Label::new(job).wrap());
            });
            continue;
        }

        // Numbered list (e.g. "1. ")
        if let Some((num, rest)) = split_numbered_list(trimmed) {
            ui.horizontal_top(|ui| {
                ui.add_space(6.0);
                ui.label(
                    egui::RichText::new(num)
                        .strong()
                        .color(egui::Color32::from_rgb(110, 185, 255)),
                );
                let mut job = egui::text::LayoutJob::default();
                append_inline_md(
                    &mut job,
                    rest,
                    13.5,
                    egui::Color32::from_rgb(230, 236, 246),
                    false,
                );
                ui.add(egui::Label::new(job).wrap());
            });
            continue;
        }

        // Regular paragraph line
        let mut job = egui::text::LayoutJob::default();
        append_inline_md(
            &mut job,
            trimmed,
            13.5,
            egui::Color32::from_rgb(230, 236, 246),
            false,
        );
        ui.add(egui::Label::new(job).wrap());
    }

    if in_code {
        flush_code(ui, &mut code_lines);
    }
}

fn strip_heading(line: &str) -> Option<&str> {
    let hashes = line.bytes().take_while(|&b| b == b'#').count();
    if (1..=4).contains(&hashes) {
        line[hashes..].strip_prefix(' ')
    } else {
        None
    }
}

fn split_numbered_list(line: &str) -> Option<(&str, &str)> {
    let digits = line.bytes().take_while(|b| b.is_ascii_digit()).count();
    if digits > 0 && digits <= 3 && line.as_bytes().get(digits) == Some(&b'.') && line.as_bytes().get(digits + 1) == Some(&b' ') {
        Some((&line[..=digits], &line[digits + 2..]))
    } else {
        None
    }
}

fn append_inline_md(
    job: &mut egui::text::LayoutJob,
    text: &str,
    size: f32,
    base_color: egui::Color32,
    default_bold: bool,
) {
    let normal_fmt = || egui::TextFormat {
        font_id: egui::FontId::proportional(size),
        color: if default_bold { egui::Color32::WHITE } else { base_color },
        ..Default::default()
    };
    let bold_fmt = || egui::TextFormat {
        font_id: egui::FontId::proportional(size),
        color: egui::Color32::WHITE,
        ..Default::default()
    };
    let italic_fmt = || egui::TextFormat {
        font_id: egui::FontId::proportional(size),
        color: egui::Color32::from_rgb(205, 220, 238),
        italics: true,
        ..Default::default()
    };
    let code_fmt = || egui::TextFormat {
        font_id: egui::FontId::monospace((size - 1.0).max(11.0)),
        color: egui::Color32::from_rgb(155, 215, 255),
        background: egui::Color32::from_rgb(12, 20, 34),
        ..Default::default()
    };
    let link_fmt = || egui::TextFormat {
        font_id: egui::FontId::proportional(size),
        color: egui::Color32::from_rgb(105, 185, 255),
        underline: egui::Stroke::new(1.0, egui::Color32::from_rgb(105, 185, 255)),
        ..Default::default()
    };

    let bytes = text.as_bytes();
    let mut i = 0usize;
    let mut seg_start = 0usize;

    let flush_plain = |job: &mut egui::text::LayoutJob, start: usize, end: usize| {
        if end > start {
            job.append(&text[start..end], 0.0, normal_fmt());
        }
    };

    while i < bytes.len() {
        // Inline code: `...`
        if bytes[i] == b'`' {
            if let Some(rel_end) = text[i + 1..].find('`') {
                flush_plain(job, seg_start, i);
                job.append(&text[i + 1..i + 1 + rel_end], 0.0, code_fmt());
                i = i + 2 + rel_end;
                seg_start = i;
                continue;
            }
        }

        // Bold: **...**
        if bytes[i] == b'*' && bytes.get(i + 1) == Some(&b'*') {
            if let Some(rel_end) = text[i + 2..].find("**") {
                flush_plain(job, seg_start, i);
                job.append(&text[i + 2..i + 2 + rel_end], 0.0, bold_fmt());
                i = i + 4 + rel_end;
                seg_start = i;
                continue;
            }
        }

        // Italic: *...*
        if bytes[i] == b'*' {
            if let Some(rel_end) = text[i + 1..].find('*') {
                if rel_end > 0 {
                    flush_plain(job, seg_start, i);
                    job.append(&text[i + 1..i + 1 + rel_end], 0.0, italic_fmt());
                    i = i + 2 + rel_end;
                    seg_start = i;
                    continue;
                }
            }
        }

        // Link: [label](https://...)
        if bytes[i] == b'[' {
            if let Some(close_br) = text[i + 1..].find("](") {
                let after = i + 1 + close_br + 2;
                if let Some(close_paren) = text[after..].find(')') {
                    let label = &text[i + 1..i + 1 + close_br];
                    let url = &text[after..after + close_paren];
                    if url.starts_with("http://") || url.starts_with("https://") {
                        flush_plain(job, seg_start, i);
                        job.append(label, 0.0, link_fmt());
                        i = after + close_paren + 1;
                        seg_start = i;
                        continue;
                    }
                }
            }
        }

        i += 1;
    }

    flush_plain(job, seg_start, bytes.len());
}
