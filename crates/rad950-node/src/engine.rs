//! Normative RAD950 v1 / RAD950-60 QSO, Hunter, and Hourly UTC Rendezvous engine.

use std::collections::{HashMap, HashSet};
use urad950::frame::{decode_grid4, encode_grid4, Callsign, Frame, GRID_UNKNOWN};
use urad950::SUBCHANNELS_HZ;

pub const MAX_TRIES: u32 = 4;
pub const EXPIRY_SLOTS: u64 = 30;
pub const LINGER_SLOTS: u64 = 2;
pub const COOLDOWN_SLOTS: u64 = 30;
pub const MAX_COOL_RETRIES: u32 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpMode {
    Monitor,
    AutoCqEven,
    AutoCqOdd,
    Pounce,
}

impl OpMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Monitor => "Monitor",
            Self::AutoCqEven => "Run (Even)",
            Self::AutoCqOdd => "Run (Odd)",
            Self::Pounce => "Pounce",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutoAnswer {
    Off,
    NewGrid,
    NewBandGrid,
    AnyNeeded,
    /// Auto-Pounce: stay silent (no CQs) and answer any incoming CQ not on cooldown.
    AllCqs,
}

impl AutoAnswer {
    pub fn parse(s: &str) -> Result<Self, String> {
        match s.trim().to_ascii_lowercase().as_str() {
            "off" | "none" => Ok(Self::Off),
            "new-grid" | "newgrid" | "grid" => Ok(Self::NewGrid),
            "new-band" | "newband" | "band" => Ok(Self::NewBandGrid),
            "any" | "any-needed" => Ok(Self::AnyNeeded),
            "all" | "all-cqs" | "auto-pounce" | "pounce" => Ok(Self::AllCqs),
            other => Err(format!(
                "invalid auto-answer mode '{other}' (expected off|new-grid|new-band|any|all-cqs)"
            )),
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Off => Self::NewGrid,
            Self::NewGrid => Self::NewBandGrid,
            Self::NewBandGrid => Self::AnyNeeded,
            Self::AnyNeeded => Self::AllCqs,
            Self::AllCqs => Self::Off,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::NewGrid => "New Grid",
            Self::NewBandGrid => "New Band+Grid",
            Self::AnyNeeded => "Any Needed",
            Self::AllCqs => "All CQs (Auto-Pounce)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Band {
    B10m,
    B12m,
    B15m,
    B17m,
    B20m,
    B40m,
    Other,
}

impl Band {
    pub fn from_dial_hz(hz: u64) -> Self {
        match hz {
            28_000_000..=29_700_000 => Self::B10m,
            24_890_000..=24_990_000 => Self::B12m,
            21_000_000..=21_450_000 => Self::B15m,
            18_068_000..=18_168_000 => Self::B17m,
            14_000_000..=14_350_000 => Self::B20m,
            7_000_000..=7_300_000 => Self::B40m,
            _ => Self::Other,
        }
    }

    pub fn parse(s: &str) -> Option<(Self, u64)> {
        match s.trim().to_ascii_lowercase().as_str() {
            "10m" => Some((Self::B10m, 28_078_000)),
            "12m" => Some((Self::B12m, 24_919_000)),
            "15m" => Some((Self::B15m, 21_078_000)),
            "17m" => Some((Self::B17m, 18_104_000)),
            "20m" => Some((Self::B20m, 14_084_000)),
            "40m" => Some((Self::B40m, 7_084_000)),
            _ => None,
        }
    }

    pub fn dial_hz(self) -> u64 {
        match self {
            Self::B10m => 28_078_000,
            Self::B12m => 24_919_000,
            Self::B15m => 21_078_000,
            Self::B17m => 18_104_000,
            Self::B20m => 14_084_000,
            Self::B40m => 7_084_000,
            Self::Other => 18_104_000,
        }
    }

    /// True for the 4 HF bands where the Radtel RT-950 Pro handheld can transmit (`10m..17m`).
    /// `20m` and `40m` are RX-only on the RT-950 Pro and full TX/RX on desktop HF transceivers.
    pub fn is_rt950_tx(self) -> bool {
        matches!(self, Self::B10m | Self::B12m | Self::B15m | Self::B17m)
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::B10m => "10m",
            Self::B12m => "12m",
            Self::B15m => "15m",
            Self::B17m => "17m",
            Self::B20m => "20m",
            Self::B40m => "40m",
            Self::Other => "OOB",
        }
    }
}

pub const RENDEZVOUS_SCHEDULE: [(Band, u64, u64); 4] = [
    (Band::B10m, 0, 28_078_000),
    (Band::B12m, 15, 24_919_000),
    (Band::B15m, 30, 21_078_000),
    (Band::B17m, 45, 18_104_000),
];

#[derive(Clone, Debug)]
pub struct RxFrameEvent {
    pub frame: Frame,
    pub rx_snr_db: i8,
    pub audio_hz: u16,
    pub slot: u64,
}

#[derive(Clone, Debug)]
pub struct TxPlan {
    pub frame: Frame,
    pub subch: usize,
    pub audio_hz: u16,
    pub summary: String,
    pub peer: Option<Callsign>,
    pub is_ack: bool,
    pub tx_snr_db: i8,
}

#[derive(Clone, Debug)]
pub struct CompletedQso {
    pub utc_sec: u64,
    pub dial_hz: u64,
    pub band: Band,
    pub peer: String,
    pub grid: String,
    pub rx_snr: i8,
    pub tx_snr: i8,
    pub mode60: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct RendezvousAction {
    pub window_open: bool,
    pub should_qsy: bool,
    pub band: Band,
    pub dial_hz: u64,
}

#[derive(Clone, Debug)]
struct PeerSession {
    peer: Callsign,
    grid: String,
    last_rx_snr: i8,
    peer_rcvd_snr: i8,
    last_rx_slot: u64,
    tries: u32,
    send_ack: bool,
    tx_r0_done_at: Option<u64>,
}

#[derive(Clone, Debug)]
struct CooldownEntry {
    until_slot: u64,
    linger_until: u64,
    retries: u32,
    last_rx_snr: i8,
    last_rx_slot: u64,
    pending_retry: bool,
}

#[derive(Clone, Debug)]
struct HunterCandidate {
    peer: Callsign,
    grid: String,
    rx_snr: i8,
    cq_slot: u64,
    priority: u8,
}

#[inline]
pub fn mix32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7FEB_352D);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846C_A68B);
    x ^ (x >> 16)
}

pub fn nearest_subchannel_index(audio_hz: u16) -> usize {
    let mut best_i = 0usize;
    let mut best_d = (audio_hz as i32 - SUBCHANNELS_HZ[0] as i32).abs();
    for (i, &hz) in SUBCHANNELS_HZ.iter().enumerate().skip(1) {
        let d = (audio_hz as i32 - hz as i32).abs();
        if d < best_d {
            best_d = d;
            best_i = i;
        }
    }
    best_i
}

pub fn parse_call_and_suffix(s: &str) -> Result<(Callsign, u8, String), String> {
    let up = s.trim().to_ascii_uppercase();
    let (base, suf) = if let Some((b, rest)) = up.split_once('-') {
        let n: u8 = rest
            .parse()
            .map_err(|_| format!("invalid callsign suffix in '{s}'"))?;
        if n > 15 {
            return Err(format!("callsign suffix must be 0..=15, got '{n}'"));
        }
        (b, n)
    } else {
        (up.as_str(), 0u8)
    };
    let call = Callsign::encode(base.as_bytes())
        .map_err(|_| format!("invalid RAD950 base callsign '{base}'"))?;
    let disp = if suf > 0 {
        format!("{base}-{suf}")
    } else {
        base.to_string()
    };
    Ok((call, suf, disp))
}

pub fn format_call(c: Callsign) -> String {
    let mut buf = [0u8; 6];
    let n = c.decode(&mut buf);
    String::from_utf8_lossy(&buf[..n]).into_owned()
}

pub fn format_grid(g: u16) -> String {
    if g >= GRID_UNKNOWN {
        return "----".to_string();
    }
    match decode_grid4(g) {
        Some(b) => String::from_utf8_lossy(&b).into_owned(),
        None => "----".to_string(),
    }
}

pub fn parse_grid(s: &str) -> Result<(u16, String), String> {
    let up = s.trim().to_ascii_uppercase();
    if up.is_empty() || up == "NONE" || up == "----" {
        return Ok((GRID_UNKNOWN, "----".to_string()));
    }
    if up.len() < 4 {
        return Err(format!("invalid 4-char grid '{s}'"));
    }
    let bytes = up.as_bytes();
    let g4 = [bytes[0], bytes[1], bytes[2], bytes[3]];
    let code = encode_grid4(&g4).ok_or_else(|| format!("invalid Maidenhead grid '{s}'"))?;
    Ok((code, String::from_utf8_lossy(&g4).into_owned()))
}

pub struct NodeEngine {
    pub my_call: Callsign,
    pub my_suffix: u8,
    pub my_call_disp: String,
    pub my_grid4: u16,
    pub my_grid_str: String,
    pub dial_hz: u64,
    pub band: Band,
    pub preferred_subch: usize,
    pub mode: OpMode,
    pub auto_answer: AutoAnswer,
    pub rdv_follow: bool,
    pub rdv_call_listen: bool,
    pub mode60: bool,

    rdv_seed: u32,
    rdv_window_was_open: bool,
    promoted_from_monitor: bool,
    last_tx_slot: Option<u64>,
    cur_rx_slot: u64,
    cur_rx_busy_mask: u8,
    last_rx_busy_mask: u8,

    sessions: HashMap<Callsign, PeerSession>,
    cooldowns: HashMap<Callsign, CooldownEntry>,
    heard_grids: HashMap<Callsign, String>,
    best_hunter: Option<HunterCandidate>,

    pub worked_grids_all: HashSet<String>,
    pub worked_grids_band: HashSet<(Band, String)>,
    pub worked_calls_band: HashSet<(Band, String)>,
}

impl NodeEngine {
    pub fn new(call: Callsign, suffix: u8, call_disp: String, grid4: u16, grid_str: String, dial_hz: u64) -> Self {
        let mut eng = Self {
            my_call: call,
            my_suffix: suffix,
            my_call_disp: call_disp,
            my_grid4: grid4,
            my_grid_str: grid_str,
            dial_hz,
            band: Band::from_dial_hz(dial_hz),
            preferred_subch: 4, // 1500 Hz
            mode: OpMode::Monitor,
            auto_answer: AutoAnswer::Off,
            rdv_follow: false,
            rdv_call_listen: false,
            mode60: false,
            rdv_seed: 0x1357_9BDF,
            rdv_window_was_open: false,
            promoted_from_monitor: false,
            last_tx_slot: None,
            cur_rx_slot: 0,
            cur_rx_busy_mask: 0,
            last_rx_busy_mask: 0,
            sessions: HashMap::new(),
            cooldowns: HashMap::new(),
            heard_grids: HashMap::new(),
            best_hunter: None,
            worked_grids_all: HashSet::new(),
            worked_grids_band: HashSet::new(),
            worked_calls_band: HashSet::new(),
        };
        eng.recompute_seed();
        eng
    }

    pub fn set_station(&mut self, call: Callsign, suffix: u8, call_disp: String, grid4: u16, grid_str: String) {
        self.my_call = call;
        self.my_suffix = suffix;
        self.my_call_disp = call_disp;
        self.my_grid4 = grid4;
        self.my_grid_str = grid_str;
        self.recompute_seed();
    }

    fn recompute_seed(&mut self) {
        let raw = self.my_call.raw() ^ ((self.my_suffix as u32) << 24);
        let s = raw.wrapping_mul(0x9E37_79B1) ^ 0x5A3C_9E37;
        self.rdv_seed = if s == 0 { 0x1357_9BDF } else { s };
    }

    pub fn set_dial_hz(&mut self, dial_hz: u64) {
        let new_band = Band::from_dial_hz(dial_hz);
        self.dial_hz = dial_hz;
        if new_band != self.band {
            self.band = new_band;
            self.reset_sessions();
        }
    }

    pub fn set_mode60(&mut self, mode60: bool) {
        if self.mode60 != mode60 {
            self.mode60 = mode60;
            self.reset_sessions();
        }
    }

    pub fn set_mode(&mut self, mode: OpMode) {
        self.mode = mode;
        self.promoted_from_monitor = false;
        if mode == OpMode::Monitor {
            self.sessions.clear();
            self.best_hunter = None;
        }
    }

    pub fn reset_sessions(&mut self) {
        self.sessions.clear();
        self.cooldowns.clear();
        self.best_hunter = None;
        self.last_tx_slot = None;
        self.cur_rx_busy_mask = 0;
        self.last_rx_busy_mask = 0;
        if self.promoted_from_monitor {
            self.mode = OpMode::Monitor;
            self.promoted_from_monitor = false;
        }
    }

    pub fn record_worked(&mut self, peer_call: &str, grid4: &str, band: Band) {
        let base = peer_call
            .trim()
            .split('-')
            .next()
            .unwrap_or("")
            .to_ascii_uppercase();
        if base.is_empty() {
            return;
        }
        self.worked_calls_band.insert((band, base));
        let g = grid4.trim().to_ascii_uppercase();
        if g.len() >= 4 && &g[..4] != "----" {
            let g4 = g[..4].to_string();
            self.worked_grids_all.insert(g4.clone());
            self.worked_grids_band.insert((band, g4));
        }
    }

    pub fn has_active_qso(&self, current_slot: u64) -> bool {
        if self
            .sessions
            .values()
            .any(|s| s.tries < MAX_TRIES && current_slot <= s.last_rx_slot.saturating_add(EXPIRY_SLOTS))
        {
            return true;
        }
        self.cooldowns.values().any(|c| current_slot <= c.linger_until)
    }

    pub fn pounce_peer(&mut self, peer: Callsign, grid: String, rx_snr: i8, heard_slot: u64, peer_sent_rx0: bool) -> bool {
        if peer == self.my_call {
            return false;
        }
        if grid.len() == 4 && grid != "----" {
            self.heard_grids.insert(peer, grid.clone());
        }
        let g = if grid.len() == 4 && grid != "----" {
            grid
        } else {
            self.heard_grids.get(&peer).cloned().unwrap_or_else(|| "----".to_string())
        };
        let sess = self.sessions.entry(peer).or_insert(PeerSession {
            peer,
            grid: g.clone(),
            last_rx_snr: rx_snr.clamp(-24, 7),
            peer_rcvd_snr: -10,
            last_rx_slot: heard_slot,
            tries: 0,
            send_ack: peer_sent_rx0,
            tx_r0_done_at: None,
        });
        sess.grid = g;
        sess.last_rx_snr = rx_snr.clamp(-24, 7);
        sess.last_rx_slot = heard_slot;
        sess.tries = 0;
        sess.send_ack = peer_sent_rx0;
        if self.mode == OpMode::Monitor {
            self.mode = OpMode::Pounce;
            self.promoted_from_monitor = false;
        }
        true
    }

    pub fn begin_rx_slot(&mut self, slot: u64) {
        if slot != self.cur_rx_slot {
            self.last_rx_busy_mask = self.cur_rx_busy_mask;
            self.cur_rx_busy_mask = 0;
            self.cur_rx_slot = slot;
            self.best_hunter = None;
        }
    }

    fn hunter_priority(&self, peer_str: &str, grid: &str) -> u8 {
        if self.auto_answer == AutoAnswer::Off {
            return 0;
        }
        let already_worked = self.worked_calls_band.contains(&(self.band, peer_str.to_string()));
        if already_worked && self.auto_answer != AutoAnswer::AllCqs {
            return 0;
        }
        let valid_g = grid.len() == 4 && grid != "----";
        let new_grid = valid_g && !self.worked_grids_all.contains(grid);
        let new_band = valid_g && !self.worked_grids_band.contains(&(self.band, grid.to_string()));
        match self.auto_answer {
            AutoAnswer::Off => 0,
            AutoAnswer::NewGrid => if new_grid { 3 } else { 0 },
            AutoAnswer::NewBandGrid => {
                if new_grid { 3 } else if new_band { 2 } else { 0 }
            }
            AutoAnswer::AnyNeeded => {
                if new_grid { 3 } else if new_band { 2 } else { 1 }
            }
            AutoAnswer::AllCqs => {
                if new_grid { 4 } else if new_band { 3 } else if !already_worked { 2 } else { 1 }
            }
        }
    }

    pub fn on_rx_frame(&mut self, ev: &RxFrameEvent, utc_sec: u64) -> Option<CompletedQso> {
        if ev.slot != self.cur_rx_slot {
            self.begin_rx_slot(ev.slot);
        }
        let sub = nearest_subchannel_index(ev.audio_hz);
        self.cur_rx_busy_mask |= 1u8 << sub;

        match ev.frame {
            Frame::Cq { sender, grid4, .. } => {
                let grid = format_grid(grid4);
                if grid != "----" {
                    self.heard_grids.insert(sender, grid.clone());
                }
                if sender == self.my_call {
                    return None;
                }
                if self.cooldowns.get(&sender).is_some_and(|c| ev.slot <= c.until_slot) {
                    return None;
                }
                if self.sessions.contains_key(&sender) {
                    return None;
                }
                if self.auto_answer != AutoAnswer::Off && !self.has_active_qso(ev.slot) {
                    let sender_str = format_call(sender);
                    let prio = self.hunter_priority(&sender_str, &grid);
                    if prio > 0 {
                        let better = self
                            .best_hunter
                            .as_ref()
                            .is_none_or(|b| (prio, ev.rx_snr_db) > (b.priority, b.rx_snr));
                        if better {
                            self.best_hunter = Some(HunterCandidate {
                                peer: sender,
                                grid,
                                rx_snr: ev.rx_snr_db.clamp(-24, 7),
                                cq_slot: ev.slot,
                                priority: prio,
                            });
                        }
                    }
                }
                None
            }
            Frame::Rx {
                sender,
                recipient,
                snr_db,
                ack,
                ..
            } => {
                if sender == self.my_call || recipient != self.my_call {
                    return None;
                }
                if !ack {
                    if let Some(cd) = self.cooldowns.get_mut(&sender) {
                        if ev.slot <= cd.until_slot {
                            if cd.retries < MAX_COOL_RETRIES {
                                cd.last_rx_snr = ev.rx_snr_db.clamp(-24, 7);
                                cd.last_rx_slot = ev.slot;
                                cd.pending_retry = true;
                            }
                            return None;
                        }
                    }
                    let can_answer = matches!(self.mode, OpMode::AutoCqEven | OpMode::AutoCqOdd | OpMode::Pounce)
                        || self.auto_answer != AutoAnswer::Off
                        || self.rdv_call_listen;
                    if !can_answer {
                        return None;
                    }
                    if self.mode == OpMode::Monitor {
                        self.mode = OpMode::Pounce;
                        self.promoted_from_monitor = true;
                    }
                    let g = self
                        .heard_grids
                        .get(&sender)
                        .cloned()
                        .unwrap_or_else(|| "----".to_string());
                    let sess = self.sessions.entry(sender).or_insert(PeerSession {
                        peer: sender,
                        grid: g,
                        last_rx_snr: ev.rx_snr_db.clamp(-24, 7),
                        peer_rcvd_snr: snr_db.clamp(-24, 7),
                        last_rx_slot: ev.slot,
                        tries: 0,
                        send_ack: true,
                        tx_r0_done_at: None,
                    });
                    sess.last_rx_snr = ev.rx_snr_db.clamp(-24, 7);
                    sess.peer_rcvd_snr = snr_db.clamp(-24, 7);
                    sess.last_rx_slot = ev.slot;
                    sess.send_ack = true;
                    sess.tries = 0;
                    None
                } else {
                    let Some(sess) = self.sessions.get(&sender) else {
                        return None;
                    };
                    let Some(r0_slot) = sess.tx_r0_done_at else {
                        return None;
                    };
                    if ev.slot > r0_slot.saturating_add(EXPIRY_SLOTS) {
                        self.sessions.remove(&sender);
                        return None;
                    }
                    let grid = if sess.grid != "----" {
                        sess.grid.clone()
                    } else {
                        self.heard_grids
                            .get(&sender)
                            .cloned()
                            .unwrap_or_else(|| "----".to_string())
                    };
                    let peer_str = format_call(sender);
                    let qso = CompletedQso {
                        utc_sec,
                        dial_hz: self.dial_hz,
                        band: self.band,
                        peer: peer_str.clone(),
                        grid: grid.clone(),
                        rx_snr: ev.rx_snr_db.clamp(-24, 7),
                        tx_snr: snr_db.clamp(-24, 7),
                        mode60: self.mode60,
                    };
                    self.record_worked(&peer_str, &grid, self.band);
                    self.sessions.remove(&sender);
                    self.cooldowns.insert(
                        sender,
                        CooldownEntry {
                            until_slot: ev.slot + COOLDOWN_SLOTS,
                            linger_until: ev.slot,
                            retries: MAX_COOL_RETRIES,
                            last_rx_snr: qso.rx_snr,
                            last_rx_slot: ev.slot,
                            pending_retry: false,
                        },
                    );
                    if self.mode == OpMode::Pounce && self.sessions.is_empty() {
                        self.mode = OpMode::Monitor;
                        self.promoted_from_monitor = false;
                    }
                    Some(qso)
                }
            }
            Frame::ExtFrag { .. } => None,
        }
    }

    fn expire_stale(&mut self, slot: u64) {
        self.sessions.retain(|_, s| {
            let rx_ok = slot <= s.last_rx_slot.saturating_add(EXPIRY_SLOTS);
            let r0_ok = s.tx_r0_done_at.is_some_and(|t| slot <= t.saturating_add(EXPIRY_SLOTS));
            rx_ok || r0_ok
        });
        self.cooldowns.retain(|_, c| slot <= c.until_slot);
        if self.promoted_from_monitor && !self.has_active_qso(slot) {
            self.mode = OpMode::Monitor;
            self.promoted_from_monitor = false;
        }
    }

    pub fn evaluate_rendezvous(&mut self, slot: u64, utc_sec: u64) -> RendezvousAction {
        let min = (utc_sec / 60) % 60;
        let active = RENDEZVOUS_SCHEDULE
            .iter()
            .find(|&&(_, start, _)| min >= start && min < start + 5);
        if let Some(&(band, _, dial_hz)) = active {
            self.rdv_window_was_open = true;
            let should_qsy = self.rdv_follow && self.band != band && !self.has_active_qso(slot);
            RendezvousAction {
                window_open: true,
                should_qsy,
                band,
                dial_hz,
            }
        } else {
            if self.rdv_window_was_open {
                self.rdv_call_listen = false;
                self.rdv_window_was_open = false;
            }
            RendezvousAction {
                window_open: false,
                should_qsy: false,
                band: Band::Other,
                dial_hz: 0,
            }
        }
    }

    pub fn pick_subchannel(&self, busy_mask: u8, seed: u32, prefer_configured: bool) -> usize {
        let pref = self.preferred_subch.min(SUBCHANNELS_HZ.len() - 1);
        if prefer_configured && (busy_mask & (1u8 << pref)) == 0 {
            return pref;
        }
        let mut clear = [0usize; 8];
        let mut n = 0usize;
        for i in 0..SUBCHANNELS_HZ.len() {
            if (busy_mask & (1u8 << i)) == 0 {
                clear[n] = i;
                n += 1;
            }
        }
        if n == 0 {
            (seed as usize) % SUBCHANNELS_HZ.len()
        } else {
            clear[(seed as usize) % n]
        }
    }

    pub fn occupied_subch_mask(&self) -> u8 {
        self.last_rx_busy_mask
    }

    pub fn plan_tx(&mut self, slot: u64, utc_sec: u64) -> Option<TxPlan> {
        self.expire_stale(slot);
        let busy_mask = if self.cur_rx_slot + 1 == slot {
            self.cur_rx_busy_mask
        } else {
            self.last_rx_busy_mask
        };

        if self.last_tx_slot.is_some_and(|prev| slot < prev + 2) {
            return None;
        }

        let rdv = self.evaluate_rendezvous(slot, utc_sec);
        if rdv.should_qsy {
            return None;
        }

        let sub_seed = mix32(self.rdv_seed ^ (slot as u32).wrapping_mul(0x9E37_79B1) ^ 0x85EB_CA6B);

        // Priority 1a: Cooldown R+RX retry
        if let Some((&peer, cd)) = self.cooldowns.iter().find(|(_, c)| {
            c.pending_retry && c.retries < MAX_COOL_RETRIES && ((c.last_rx_slot + 1) % 2 == slot % 2)
        }) {
            let subch = self.pick_subchannel(busy_mask, sub_seed, true);
            let snr = cd.last_rx_snr;
            let frame = Frame::Rx {
                sender: self.my_call,
                recipient: peer,
                snr_db: snr,
                ack: true,
                reserved: false,
            };
            return Some(TxPlan {
                frame,
                subch,
                audio_hz: SUBCHANNELS_HZ[subch],
                summary: format!("R+RX {} {} {:+03}", self.my_call_disp, format_call(peer), snr),
                peer: Some(peer),
                is_ack: true,
                tx_snr_db: snr,
            });
        }

        // Priority 1b: Pending directed peer session (ACK=1 before RX=0)
        let chosen = [true, false].into_iter().find_map(|want_ack| {
            self.sessions
                .values()
                .filter(|s| {
                    s.send_ack == want_ack
                        && s.tries < MAX_TRIES
                        && ((s.last_rx_slot + 1) % 2 == slot % 2)
                })
                .min_by_key(|s| s.last_rx_slot)
                .cloned()
        });
        if let Some(s) = chosen {
            let subch = self.pick_subchannel(busy_mask, sub_seed, true);
            let frame = Frame::Rx {
                sender: self.my_call,
                recipient: s.peer,
                snr_db: s.last_rx_snr,
                ack: s.send_ack,
                reserved: false,
            };
            let tag = if s.send_ack { "R+RX" } else { "RX" };
            return Some(TxPlan {
                frame,
                subch,
                audio_hz: SUBCHANNELS_HZ[subch],
                summary: format!("{tag} {} {} {:+03}", self.my_call_disp, format_call(s.peer), s.last_rx_snr),
                peer: Some(s.peer),
                is_ack: s.send_ack,
                tx_snr_db: s.last_rx_snr,
            });
        }

        // Priority 2: Hunter Auto-Answer candidate
        if let Some(cand) = self.best_hunter.as_ref() {
            if (cand.cq_slot + 1) % 2 == slot % 2 {
                let cand = self.best_hunter.take().unwrap();
                self.pounce_peer(cand.peer, cand.grid, cand.rx_snr, cand.cq_slot, false);
                let subch = self.pick_subchannel(busy_mask, sub_seed, true);
                let frame = Frame::Rx {
                    sender: self.my_call,
                    recipient: cand.peer,
                    snr_db: cand.rx_snr,
                    ack: false,
                    reserved: false,
                };
                return Some(TxPlan {
                    frame,
                    subch,
                    audio_hz: SUBCHANNELS_HZ[subch],
                    summary: format!("RX {} {} {:+03}", self.my_call_disp, format_call(cand.peer), cand.rx_snr),
                    peer: Some(cand.peer),
                    is_ack: false,
                    tx_snr_db: cand.rx_snr,
                });
            }
        }

        // Priority 3: Run / Auto-CQ
        if matches!(self.mode, OpMode::AutoCqEven | OpMode::AutoCqOdd) && !self.has_active_qso(slot) {
            let want_parity = if self.mode == OpMode::AutoCqEven { 0 } else { 1 };
            if slot % 2 == want_parity {
                let subch = self.pick_subchannel(busy_mask, sub_seed, true);
                let frame = Frame::Cq {
                    sender: self.my_call,
                    grid4: self.my_grid4,
                    tx_pwr: 10,
                    reserved: (self.my_suffix & 0x0F) as u16,
                };
                return Some(TxPlan {
                    frame,
                    subch,
                    audio_hz: SUBCHANNELS_HZ[subch],
                    summary: format!("CQ {} {}", self.my_call_disp, self.my_grid_str),
                    peer: None,
                    is_ack: false,
                    tx_snr_db: 0,
                });
            }
        }

        // Priority 4: Hourly UTC Rendezvous adaptive 50/25/12.5% Call & Listen (even slots only)
        if self.rdv_call_listen && rdv.window_open && slot % 2 == 0 && !self.has_active_qso(slot) {
            let r = mix32(self.rdv_seed ^ (slot as u32).wrapping_mul(0x9E37_79B1));
            let busy_cnt = busy_mask.count_ones();
            let mask_p = if busy_cnt < 4 { 1 } else if busy_cnt <= 5 { 3 } else { 7 };
            if (r & mask_p) == 1 {
                let sub_r = mix32(r ^ 0x85EB_CA6B);
                let subch = self.pick_subchannel(busy_mask, sub_r, false);
                let frame = Frame::Cq {
                    sender: self.my_call,
                    grid4: self.my_grid4,
                    tx_pwr: 10,
                    reserved: (self.my_suffix & 0x0F) as u16,
                };
                return Some(TxPlan {
                    frame,
                    subch,
                    audio_hz: SUBCHANNELS_HZ[subch],
                    summary: format!("RDV CQ {} {}", self.my_call_disp, self.my_grid_str),
                    peer: None,
                    is_ack: false,
                    tx_snr_db: 0,
                });
            }
        }

        None
    }

    pub fn confirm_tx_completed(&mut self, slot: u64, utc_sec: u64, plan: &TxPlan) -> Option<CompletedQso> {
        self.last_tx_slot = Some(slot);
        let peer = plan.peer?;

        if let Some(cd) = self.cooldowns.get_mut(&peer) {
            if plan.is_ack && cd.pending_retry {
                cd.pending_retry = false;
                cd.retries += 1;
                return None;
            }
        }

        let sess = self.sessions.get_mut(&peer)?;
        sess.tries += 1;
        if !plan.is_ack {
            sess.tx_r0_done_at = Some(slot);
            None
        } else {
            let sess = self.sessions.remove(&peer)?;
            let grid = if sess.grid != "----" {
                sess.grid
            } else {
                self.heard_grids
                    .get(&peer)
                    .cloned()
                    .unwrap_or_else(|| "----".to_string())
            };
            let peer_str = format_call(peer);
            let qso = CompletedQso {
                utc_sec,
                dial_hz: self.dial_hz,
                band: self.band,
                peer: peer_str.clone(),
                grid: grid.clone(),
                rx_snr: sess.last_rx_snr,
                tx_snr: sess.peer_rcvd_snr,
                mode60: self.mode60,
            };
            self.record_worked(&peer_str, &grid, self.band);
            self.cooldowns.insert(
                peer,
                CooldownEntry {
                    until_slot: slot + COOLDOWN_SLOTS,
                    linger_until: slot + LINGER_SLOTS,
                    retries: 0,
                    last_rx_snr: plan.tx_snr_db,
                    last_rx_slot: slot,
                    pending_retry: false,
                },
            );
            if self.promoted_from_monitor && self.sessions.is_empty() {
                self.mode = OpMode::Monitor;
                self.promoted_from_monitor = false;
            }
            Some(qso)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_slot_qso_and_pileup_retention() {
        let (c1, s1, d1) = parse_call_and_suffix("NQ6N").unwrap();
        let (g1, gs1) = parse_grid("EN61").unwrap();
        let mut runner = NodeEngine::new(c1, s1, d1, g1, gs1, 18_104_000);
        runner.set_mode(OpMode::AutoCqEven);

        let (c2, s2, d2) = parse_call_and_suffix("K1ABC").unwrap();
        let (g2, gs2) = parse_grid("FN42").unwrap();
        let mut hunter = NodeEngine::new(c2, s2, d2, g2, gs2, 18_104_000);
        hunter.auto_answer = AutoAnswer::NewGrid;

        // Cold ACK=1 ignored
        let cold = RxFrameEvent {
            frame: Frame::Rx { sender: c1, recipient: c2, snr_db: -6, ack: true, reserved: false },
            rx_snr_db: -5,
            audio_hz: 1500,
            slot: 10,
        };
        assert!(hunter.on_rx_frame(&cold, 100).is_none());

        // Slot 10: runner CQ
        let p10 = runner.plan_tx(10, 100).unwrap();
        assert!(runner.confirm_tx_completed(10, 100, &p10).is_none());
        hunter.on_rx_frame(&RxFrameEvent { frame: p10.frame, rx_snr_db: -7, audio_hz: p10.audio_hz, slot: 10 }, 100);

        // Slot 11: hunter RX(ACK=0)
        let p11 = hunter.plan_tx(11, 110).unwrap();
        assert!(!p11.is_ack);
        hunter.confirm_tx_completed(11, 110, &p11);
        runner.on_rx_frame(&RxFrameEvent { frame: p11.frame, rx_snr_db: -9, audio_hz: p11.audio_hz, slot: 11 }, 110);

        // Slot 12: runner R+RX(ACK=1) -> logs on clean TX completion
        let p12 = runner.plan_tx(12, 120).unwrap();
        assert!(p12.is_ack);
        let rqso = runner.confirm_tx_completed(12, 120, &p12).unwrap();
        assert_eq!(rqso.peer, "K1ABC");

        // Hunter receives causal R+RX(ACK=1) -> logs QSO
        let hqso = hunter.on_rx_frame(&RxFrameEvent { frame: p12.frame, rx_snr_db: -6, audio_hz: p12.audio_hz, slot: 12 }, 120).unwrap();
        assert_eq!(hqso.peer, "NQ6N");
        assert_eq!(hqso.grid, "EN61");
    }

    #[test]
    fn pileup_max_tries_retains_r0_session() {
        let (c1, s1, d1) = parse_call_and_suffix("W1XYZ").unwrap();
        let (g1, gs1) = parse_grid("FN31").unwrap();
        let (c_run, _, _) = parse_call_and_suffix("NQ6N").unwrap();
        let mut caller = NodeEngine::new(c1, s1, d1, g1, gs1, 21_078_000);
        caller.pounce_peer(c_run, "EN61".to_string(), -10, 20, false);

        for s in [21u64, 23, 25, 27] {
            let p = caller.plan_tx(s, s * 10).unwrap();
            caller.confirm_tx_completed(s, s * 10, &p);
        }
        assert!(caller.plan_tx(29, 290).is_none());

        // Late R+RX at slot 36 still logs cleanly!
        let late = RxFrameEvent {
            frame: Frame::Rx { sender: c_run, recipient: c1, snr_db: -12, ack: true, reserved: false },
            rx_snr_db: -11,
            audio_hz: 1050,
            slot: 36,
        };
        let qso = caller.on_rx_frame(&late, 360).unwrap();
        assert_eq!(qso.peer, "NQ6N");
    }
}
