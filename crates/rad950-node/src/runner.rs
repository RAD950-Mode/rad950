//! Unified background node runner driving either Audio+Hamlib or HackRF,
//! shared identically by both the native `egui` GUI and the `--text` `ratatui` TUI.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use urad950::frame::Frame;
use urad950::{encode_tones, Decoded, Receiver, Receiver60, MAX_DECODED, SUBCHANNELS_HZ};

use crate::adif::{append_adif_qso, load_adif_into_engine};
use crate::audio::{AudioBackend, AudioConfig};
use crate::engine::{
    format_call, format_grid, parse_call_and_suffix, parse_grid, AutoAnswer, Band, CompletedQso,
    NodeEngine, RxFrameEvent,
};
use crate::hackrf::{HackrfBackend, HackrfConfig};
use crate::spot::{post_spots, SpotItem};
use crate::tci::{TciBackend, TciConfig};

pub const WF_BINS: usize = 96; // 450..2450 Hz (~20.8 Hz/bin)
pub const WF_ROWS: usize = 48;

#[derive(Clone, Debug)]
pub enum BackendKind {
    Audio(AudioConfig),
    Hackrf(HackrfConfig),
    Tci(TciConfig),
}

#[derive(Clone, Debug)]
pub struct NodeConfig {
    pub call: String,
    pub grid: String,
    pub dial_hz: u64,
    pub subch: usize,
    pub mode60: bool,
    pub auto_answer: AutoAnswer,
    pub rdv_follow: bool,
    pub rdv_call_listen: bool,
    pub log_path: PathBuf,
    pub spot_enabled: bool,
    pub spot_url: String,
    pub spot_key: String,
    pub backend: BackendKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotPhase {
    Waiting,
    Rx,
    Tx,
    DryTx,
}

#[derive(Clone, Debug)]
pub struct HeardRow {
    pub utc_hms: String,
    pub snr_db: i8,
    pub audio_hz: u16,
    pub call: String,
    pub raw_call: urad950::frame::Callsign,
    pub grid: String,
    pub message: String,
    pub slot: u64,
    pub is_rx0_to_me: bool,
}

pub struct SharedNodeState {
    pub engine: NodeEngine,
    pub phase: SlotPhase,
    pub phase_note: String,
    pub peak_pct: f32,
    pub slots_rx: u64,
    pub slots_tx: u64,
    pub frames_rx: u64,
    pub spots_sent: u64,
    pub spots_err: u64,
    pub heard: Vec<HeardRow>,
    pub qso_log: Vec<CompletedQso>,
    pub events: VecDeque<String>,
    pub waterfall: VecDeque<[u8; WF_BINS]>,
    pub tx_enabled: bool,
    pub dry_tx: bool,
    pub spot_enabled: bool,
    pub spot_url: String,
    pub spot_key: String,
    pub log_path: PathBuf,
    pub backend_label: String,
    pub cat_status: String,
    pub pending_dial_qsy: Option<u64>,
    pub pending_backend_reload: Option<BackendKind>,
}

impl SharedNodeState {
    pub fn push_event(&mut self, msg: String) {
        self.events.push_back(msg);
        while self.events.len() > 200 {
            self.events.pop_front();
        }
    }

    pub fn push_wf_row(&mut self, row: [u8; WF_BINS]) {
        self.waterfall.push_front(row);
        while self.waterfall.len() > WF_ROWS {
            self.waterfall.pop_back();
        }
    }
}

pub struct NodeRunner {
    pub state: Arc<Mutex<SharedNodeState>>,
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl NodeRunner {
    pub fn start(cfg: NodeConfig) -> Result<Self, String> {
        let (call, suf, call_disp) = parse_call_and_suffix(&cfg.call)?;
        let (grid4, grid_str) = parse_grid(&cfg.grid)?;

        let mut engine = NodeEngine::new(call, suf, call_disp, grid4, grid_str, cfg.dial_hz);
        engine.preferred_subch = cfg.subch.min(SUBCHANNELS_HZ.len() - 1);
        engine.auto_answer = cfg.auto_answer;
        engine.rdv_follow = cfg.rdv_follow;
        engine.rdv_call_listen = cfg.rdv_call_listen;
        engine.set_mode60(cfg.mode60);

        let loaded = load_adif_into_engine(&cfg.log_path, &mut engine);

        let (dry_tx, backend_label) = backend_summary(&cfg.backend);

        let mut st = SharedNodeState {
            engine,
            phase: SlotPhase::Waiting,
            phase_note: "Syncing to UTC slot...".to_string(),
            peak_pct: 0.0,
            slots_rx: 0,
            slots_tx: 0,
            frames_rx: 0,
            spots_sent: 0,
            spots_err: 0,
            heard: Vec::new(),
            qso_log: Vec::new(),
            events: VecDeque::new(),
            waterfall: VecDeque::with_capacity(WF_ROWS),
            tx_enabled: false,
            dry_tx,
            spot_enabled: cfg.spot_enabled,
            spot_url: cfg.spot_url.clone(),
            spot_key: cfg.spot_key.clone(),
            log_path: cfg.log_path.clone(),
            backend_label,
            cat_status: "Idle".to_string(),
            pending_dial_qsy: None,
            pending_backend_reload: None,
        };
        if loaded > 0 {
            st.push_event(format!("Loaded {loaded} RAD950 worked QSOs from {}", cfg.log_path.display()));
        }

        let state = Arc::new(Mutex::new(st));
        let running = Arc::new(AtomicBool::new(true));

        let st_bg = Arc::clone(&state);
        let run_bg = Arc::clone(&running);
        let backend_cfg = cfg.backend.clone();
        let initial_dial = cfg.dial_hz;

        let handle = thread::spawn(move || {
            worker_loop(st_bg, run_bg, backend_cfg, initial_dial);
        });

        Ok(Self {
            state,
            running,
            handle: Some(handle),
        })
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for NodeRunner {
    fn drop(&mut self) {
        self.stop();
    }
}

fn backend_summary(kind: &BackendKind) -> (bool, String) {
    match kind {
        BackendKind::Audio(a) => (
            a.dry_tx,
            if a.hamlib.addr.is_empty() {
                "Audio (No CAT)".to_string()
            } else {
                format!("Audio + Hamlib ({})", a.hamlib.addr)
            },
        ),
        BackendKind::Hackrf(h) => (
            h.dry_tx,
            format!("HackRF ({})", h.serial.as_deref().unwrap_or("default")),
        ),
        BackendKind::Tci(t) => (t.dry_tx, format!("TCI ({})", t.addr)),
    }
}

enum ActiveBackend {
    Audio(AudioBackend),
    Hackrf(HackrfBackend),
    Tci(TciBackend),
}

impl ActiveBackend {
    fn open(kind: BackendKind, dial_hz: u64, state: &Arc<Mutex<SharedNodeState>>) -> Self {
        let (_, label) = backend_summary(&kind);
        state.lock().unwrap().backend_label = label;
        match kind {
            BackendKind::Audio(acfg) => {
                let mut ab = AudioBackend::new(acfg);
                let status = match ab.start_cat(dial_hz) {
                    Ok(()) => "CAT Ready".to_string(),
                    Err(e) => format!("CAT Err: {e}"),
                };
                state.lock().unwrap().cat_status = status;
                Self::Audio(ab)
            }
            BackendKind::Hackrf(hcfg) => {
                state.lock().unwrap().cat_status = "HackRF Direct".to_string();
                Self::Hackrf(HackrfBackend::new(hcfg, dial_hz))
            }
            BackendKind::Tci(tcfg) => {
                let mut tb = TciBackend::new(tcfg, dial_hz);
                let status = match tb.connect() {
                    Ok(()) => "TCI Connected".to_string(),
                    Err(e) => format!("TCI Err: {e}"),
                };
                state.lock().unwrap().cat_status = status;
                Self::Tci(tb)
            }
        }
    }

    fn set_dry_tx(&mut self, dry_tx: bool) {
        match self {
            Self::Audio(a) => a.set_dry_tx(dry_tx),
            Self::Hackrf(h) => h.set_dry_tx(dry_tx),
            Self::Tci(t) => t.set_dry_tx(dry_tx),
        }
    }

    fn set_dial_hz(&mut self, hz: u64) -> Result<(), String> {
        match self {
            Self::Audio(a) => a.set_dial_hz(hz),
            Self::Hackrf(h) => {
                h.set_dial_hz(hz);
                Ok(())
            }
            Self::Tci(t) => t.set_dial_hz(hz),
        }
    }

    fn capture_12k(&mut self, duration_sec: f64, running: &AtomicBool) -> Result<(Vec<i16>, f32), String> {
        match self {
            Self::Audio(a) => a.capture_12k(duration_sec, running),
            Self::Hackrf(h) => h.capture_12k(duration_sec, running),
            Self::Tci(t) => t.capture_12k(duration_sec, running),
        }
    }

    fn transmit_burst(
        &mut self,
        tones: &[u8; urad950::N_SYMBOLS],
        audio_hz: u16,
        running: &AtomicBool,
    ) -> Result<(), String> {
        match self {
            Self::Audio(a) => a.transmit_burst(tones, audio_hz, running),
            Self::Hackrf(h) => h.transmit_burst(tones, audio_hz, running),
            Self::Tci(t) => t.transmit_burst(tones, audio_hz, running),
        }
    }
}

fn utc_now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

pub fn format_utc_hms(utc_sec: u64) -> String {
    let tod = utc_sec % 86_400;
    format!("{:02}:{:02}:{:02}", tod / 3600, (tod % 3600) / 60, tod % 60)
}

fn worker_loop(
    state: Arc<Mutex<SharedNodeState>>,
    running: Arc<AtomicBool>,
    backend_cfg: BackendKind,
    initial_dial: u64,
) {
    let mut backend = ActiveBackend::open(backend_cfg, initial_dial, &state);

    let mut rx10 = Box::new(Receiver::new());
    let mut rx60 = Box::new(Receiver60::new());

    while running.load(Ordering::Relaxed) {
        // Apply any hot-reloaded backend from GUI Settings
        let reload_req = {
            let mut st = state.lock().unwrap();
            let req = st.pending_backend_reload.take();
            let dial = st.engine.dial_hz;
            let dry = st.dry_tx;
            (req, dial, dry)
        };
        if let Some(new_kind) = reload_req.0 {
            backend = ActiveBackend::open(new_kind, reload_req.1, &state);
            state.lock().unwrap().push_event("Reloaded radio backend".to_string());
        }
        backend.set_dry_tx(reload_req.2);

        // Apply any user-requested QSY
        let qsy_req = state.lock().unwrap().pending_dial_qsy.take();
        if let Some(hz) = qsy_req {
            if let Err(e) = backend.set_dial_hz(hz) {
                state.lock().unwrap().push_event(format!("QSY error: {e}"));
            } else {
                let mut st = state.lock().unwrap();
                st.engine.set_dial_hz(hz);
                st.push_event(format!("QSY -> {:.3} MHz ({})", hz as f64 / 1e6, Band::from_dial_hz(hz).label()));
            }
        }

        let mode60 = state.lock().unwrap().engine.mode60;
        let slot_ms_len = if mode60 { 60_000u64 } else { 10_000u64 };

        // Sleep until start of next slot
        let now_ms = utc_now_ms();
        let rem_ms = slot_ms_len - (now_ms % slot_ms_len);
        if rem_ms > 200 {
            {
                let mut st = state.lock().unwrap();
                st.phase = SlotPhase::Waiting;
                st.phase_note = format!("Next {}s UTC slot in {:.1}s", slot_ms_len / 1000, rem_ms as f64 / 1000.0);
            }
            let mut left = rem_ms;
            while left > 0 && running.load(Ordering::Relaxed) {
                let step = left.min(100);
                thread::sleep(Duration::from_millis(step));
                left -= step;
                if state.lock().unwrap().pending_dial_qsy.is_some() {
                    break;
                }
            }
            if !running.load(Ordering::Relaxed) {
                break;
            }
            if state.lock().unwrap().pending_dial_qsy.is_some() {
                continue;
            }
        }

        let slot_start_ms = utc_now_ms() + 250;
        let slot = slot_start_ms / slot_ms_len;
        let utc_sec = slot * (slot_ms_len / 1000);

        // Evaluate Rendezvous QSY + TX plan
        let (tx_plan, rdv_qsy, dry_tx) = {
            let mut st = state.lock().unwrap();
            let rdv = st.engine.evaluate_rendezvous(slot, utc_sec);
            let qsy = if rdv.should_qsy { Some(rdv.dial_hz) } else { None };
            let plan = if st.tx_enabled && qsy.is_none() {
                st.engine.plan_tx(slot, utc_sec)
            } else {
                None
            };
            (plan, qsy, st.dry_tx)
        };

        if let Some(hz) = rdv_qsy {
            let _ = backend.set_dial_hz(hz);
            let mut st = state.lock().unwrap();
            st.engine.set_dial_hz(hz);
            st.push_event(format!(
                "[{}] RDV QSY -> {:.3} MHz ({})",
                format_utc_hms(utc_sec),
                hz as f64 / 1e6,
                Band::from_dial_hz(hz).label()
            ));
            continue;
        }

        if let Some(plan) = tx_plan {
            let tones = encode_tones(plan.frame);
            {
                let mut st = state.lock().unwrap();
                st.phase = if dry_tx { SlotPhase::DryTx } else { SlotPhase::Tx };
                st.phase_note = format!("{} ({} Hz)", plan.summary, plan.audio_hz);
                st.push_event(format!(
                    "[{}] {} {}Hz  {}",
                    format_utc_hms(utc_sec),
                    if dry_tx { "DRY-TX" } else { "TX    " },
                    plan.audio_hz,
                    plan.summary
                ));
            }
            // Wait nominal +400 ms into slot
            let into = utc_now_ms() % slot_ms_len;
            if into < 400 {
                thread::sleep(Duration::from_millis(400 - into));
            }

            let mut tx_ok = true;
            if mode60 {
                for sub in 0..5u64 {
                    if !running.load(Ordering::Relaxed) {
                        tx_ok = false;
                        break;
                    }
                    if sub > 0 {
                        let wait = 10_000u64.saturating_sub(utc_now_ms() % 10_000) + 400;
                        thread::sleep(Duration::from_millis(wait.min(1600)));
                    }
                    if let Err(e) = backend.transmit_burst(&tones, plan.audio_hz, &running) {
                        state.lock().unwrap().push_event(format!("TX60 sub {sub} err: {e}"));
                        tx_ok = false;
                        break;
                    }
                }
            } else if let Err(e) = backend.transmit_burst(&tones, plan.audio_hz, &running) {
                state.lock().unwrap().push_event(format!("TX err: {e}"));
                tx_ok = false;
            }

            if tx_ok {
                let mut st = state.lock().unwrap();
                st.slots_tx += 1;
                if let Some(qso) = st.engine.confirm_tx_completed(slot, utc_sec, &plan) {
                    let _ = append_adif_qso(&st.log_path, &qso, &st.engine.my_call_disp, &st.engine.my_grid_str);
                    st.push_event(format!(
                        "[{}] QSO LOGGED: {} {} (rx {:+} / tx {:+})",
                        format_utc_hms(utc_sec),
                        qso.peer,
                        qso.grid,
                        qso.rx_snr,
                        qso.tx_snr
                    ));
                    st.qso_log.insert(0, qso);
                }
            }
        } else {
            // RX slot
            {
                let mut st = state.lock().unwrap();
                st.phase = SlotPhase::Rx;
                st.phase_note = format!(
                    "Listening {:.3} MHz ({})",
                    st.engine.dial_hz as f64 / 1e6,
                    st.engine.band.label()
                );
            }
            let cap_sec = if mode60 { 50.0 } else { 9.75 };
            match backend.capture_12k(cap_sec, &running) {
                Ok((pcm, peak_pct)) => {
                    if !running.load(Ordering::Relaxed) {
                        break;
                    }
                    let wf = compute_waterfall_row(&pcm);
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    let n_dec = if mode60 {
                        rx60.reset();
                        for sub in 0..5usize {
                            let start = sub * 120_000;
                            let end = (start + 120_000).min(pcm.len());
                            if end > start {
                                rx60.start_subslot();
                                rx60.push_samples(&pcm[start..end]);
                            }
                        }
                        rx60.finish_minute(&mut out)
                    } else {
                        rx10.reset();
                        rx10.push_samples(&pcm);
                        rx10.finish_slot(&mut out)
                    };
                    let decoded = &out[..n_dec];

                    let mut spots = Vec::new();
                    let (spot_enabled, spot_url, spot_key) = {
                        let mut st = state.lock().unwrap();
                        st.slots_rx += 1;
                        st.peak_pct = peak_pct;
                        st.push_wf_row(wf);
                        st.engine.begin_rx_slot(slot);

                        let my_call = st.engine.my_call;
                        let dial_hz = st.engine.dial_hz;
                        let band_lbl = st.engine.band.label().to_string();
                        let spotter_call = st.engine.my_call_disp.clone();
                        let spotter_grid = if st.engine.my_grid_str != "----" {
                            Some(st.engine.my_grid_str.clone())
                        } else {
                            None
                        };

                        for d in decoded {
                            st.frames_rx += 1;
                            let ev = RxFrameEvent {
                                frame: d.frame,
                                rx_snr_db: d.snr_db,
                                audio_hz: d.audio_hz,
                                slot,
                            };
                            let (call_s, raw_c, grid_s, msg_s, is_rx0_to_me, ftype, pwr_code) =
                                match d.frame {
                                    Frame::Cq {
                                        sender,
                                        grid4,
                                        tx_pwr,
                                        reserved,
                                    } => {
                                        let base = format_call(sender);
                                        let suf = (reserved & 0x0F) as u8;
                                        let disp = if suf > 0 { format!("{base}-{suf}") } else { base };
                                        let g = format_grid(grid4);
                                        (
                                            disp.clone(),
                                            Some(sender),
                                            g.clone(),
                                            format!("CQ {disp} {g}"),
                                            false,
                                            "CQ",
                                            if tx_pwr > 0 { Some(tx_pwr) } else { None },
                                        )
                                    }
                                    Frame::Rx {
                                        sender,
                                        recipient,
                                        snr_db,
                                        ack,
                                        ..
                                    } => {
                                        let sc = format_call(sender);
                                        let rc = format_call(recipient);
                                        let tag = if ack { "R+RX" } else { "RX" };
                                        (
                                            sc.clone(),
                                            Some(sender),
                                            "----".to_string(),
                                            format!("{tag} {sc} {rc} {:+03}", snr_db),
                                            !ack && recipient == my_call,
                                            tag,
                                            None,
                                        )
                                    }
                                    Frame::ExtFrag { kind, idx, tag, .. } => (
                                        "EXT".to_string(),
                                        None,
                                        "----".to_string(),
                                        format!("EXT k={kind} i={idx} t=0x{tag:02X}"),
                                        false,
                                        "EXT",
                                        None,
                                    ),
                                };

                            let hms = format_utc_hms(utc_sec);
                            st.push_event(format!("[{hms}] RX {:+3}dB {:4}Hz  {msg_s}", d.snr_db, d.audio_hz));
                            if let Some(rc) = raw_c {
                                st.heard.insert(
                                    0,
                                    HeardRow {
                                        utc_hms: hms,
                                        snr_db: d.snr_db,
                                        audio_hz: d.audio_hz,
                                        call: call_s.clone(),
                                        raw_call: rc,
                                        grid: grid_s.clone(),
                                        message: msg_s,
                                        slot,
                                        is_rx0_to_me,
                                    },
                                );
                                if st.heard.len() > 200 {
                                    st.heard.truncate(200);
                                }
                                if st.spot_enabled && rc != my_call && ftype != "EXT" {
                                    spots.push(SpotItem {
                                        spotter_call: spotter_call.clone(),
                                        spotter_grid: spotter_grid.clone(),
                                        tx_call: call_s,
                                        tx_grid: if grid_s != "----" { Some(grid_s) } else { None },
                                        utc_sec,
                                        freq_hz: dial_hz + d.audio_hz as u64,
                                        band: band_lbl.clone(),
                                        snr_db: d.snr_db,
                                        tx_pwr_code: pwr_code,
                                        frame_type: ftype.to_string(),
                                    });
                                }
                            }

                            if let Some(qso) = st.engine.on_rx_frame(&ev, utc_sec) {
                                let _ = append_adif_qso(
                                    &st.log_path,
                                    &qso,
                                    &st.engine.my_call_disp,
                                    &st.engine.my_grid_str,
                                );
                                st.push_event(format!(
                                    "[{}] QSO LOGGED: {} {} (rx {:+} / tx {:+})",
                                    format_utc_hms(utc_sec),
                                    qso.peer,
                                    qso.grid,
                                    qso.rx_snr,
                                    qso.tx_snr
                                ));
                                st.qso_log.insert(0, qso);
                            }
                        }

                        (st.spot_enabled, st.spot_url.clone(), st.spot_key.clone())
                    };

                    if spot_enabled && !spots.is_empty() && !spot_key.trim().is_empty() {
                        let st_spot = Arc::clone(&state);
                        thread::spawn(move || match post_spots(&spot_url, &spot_key, &spots) {
                            Ok(n) => st_spot.lock().unwrap().spots_sent += n as u64,
                            Err(_) => st_spot.lock().unwrap().spots_err += 1,
                        });
                    }
                }
                Err(e) => {
                    state.lock().unwrap().push_event(format!("RX err: {e}"));
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }
}

fn compute_waterfall_row(pcm: &[i16]) -> [u8; WF_BINS] {
    let mut row = [0u8; WF_BINS];
    if pcm.len() < 1024 {
        return row;
    }
    let slice = &pcm[pcm.len() / 4..(pcm.len() / 4 + 1024).min(pcm.len())];
    let n = slice.len() as f64;
    let mut vals = [0.0f64; WF_BINS];
    let mut min_db = f64::INFINITY;
    let mut max_db = f64::NEG_INFINITY;
    for (b, v) in vals.iter_mut().enumerate() {
        let hz = 450.0 + (b as f64) * (2000.0 / WF_BINS as f64);
        let w = 2.0 * std::f64::consts::PI * hz / 12_000.0;
        let coeff = 2.0 * w.cos();
        let (mut z1, mut z2) = (0.0f64, 0.0f64);
        for &s in slice {
            let z0 = (s as f64) + coeff * z1 - z2;
            z2 = z1;
            z1 = z0;
        }
        let p = (z1 * z1 + z2 * z2 - coeff * z1 * z2).max(1.0) / (n * n);
        let db = 10.0 * p.log10();
        *v = db;
        if db < min_db {
            min_db = db;
        }
        if db > max_db {
            max_db = db;
        }
    }
    let span = (max_db - min_db).max(20.0);
    for (i, &db) in vals.iter().enumerate() {
        row[i] = (((db - min_db) / span).clamp(0.0, 1.0) * 255.0) as u8;
    }
    row
}
