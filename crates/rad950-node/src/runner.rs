//! Unified background node runner driving either Audio+Hamlib or HackRF,
//! shared identically by both the native `egui` GUI and the `--text` `ratatui` TUI.

use std::collections::VecDeque;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use urad950::frame::Frame;
use urad950::{encode_tones, Decoded, Receiver, Receiver60, MAX_DECODED, SUBCHANNELS_HZ};

use crate::adif::{append_adif_qso, load_adif_records};
use crate::audio::{synth_4fsk_f32, AudioBackend, AudioConfig};
use crate::engine::{
    format_call, format_grid, parse_call_and_suffix, parse_grid, AutoAnswer, Band, CompletedQso,
    NodeEngine, RxFrameEvent,
};
use crate::hackrf::{HackrfBackend, HackrfConfig};
use crate::spectrum::{SpectrumAnalyzer, WaterfallRow, WF_BINS, WF_HOP, WF_ROWS};
use crate::spot::{post_spots, SpotItem};
use crate::tci::{TciBackend, TciConfig};

pub use crate::spectrum::{
    RowMarker as WfRowMarker, WaterfallRow as WfRow, WF_BINS as WATERFALL_BINS,
    WF_MAX_HZ as WATERFALL_MAX_HZ, WF_MIN_HZ as WATERFALL_MIN_HZ, WF_ROWS as WATERFALL_ROWS,
};

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
    pub all_txt_path: PathBuf,
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
    pub waterfall: VecDeque<WaterfallRow>,
    pub spectrum_db: [f32; WF_BINS],
    pub peak_hold_db: [f32; WF_BINS],
    pub wf_seq: u64,
    pub tx_enabled: bool,
    pub dry_tx: bool,
    pub spot_enabled: bool,
    pub spot_url: String,
    pub spot_key: String,
    pub log_path: PathBuf,
    pub all_txt_path: PathBuf,
    pub backend_label: String,
    pub cat_status: String,
    pub last_tx_summary: String,
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

    pub fn push_wf_row(&mut self, row: WaterfallRow) {
        for (i, &db10) in row.bins_db10.iter().enumerate() {
            let db = db10 as f32 / 10.0;
            // Fast attack, smooth exponential decay for live spectrum trace
            let cur = self.spectrum_db[i];
            self.spectrum_db[i] = if db > cur {
                cur * 0.35 + db * 0.65
            } else {
                cur * 0.72 + db * 0.28
            };
            // Slow-decaying peak hold (~0.35 dB per frame)
            let pk = self.peak_hold_db[i] - 0.35;
            self.peak_hold_db[i] = db.max(pk);
        }
        self.waterfall.push_front(row);
        while self.waterfall.len() > WF_ROWS {
            self.waterfall.pop_back();
        }
        self.wf_seq = self.wf_seq.wrapping_add(1);
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

        let loaded_qsos = load_adif_records(&cfg.log_path, &mut engine);
        let loaded = loaded_qsos.len();

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
            qso_log: loaded_qsos,
            events: VecDeque::new(),
            waterfall: VecDeque::with_capacity(WF_ROWS),
            spectrum_db: [-100.0; WF_BINS],
            peak_hold_db: [-100.0; WF_BINS],
            wf_seq: 0,
            tx_enabled: false,
            dry_tx,
            spot_enabled: cfg.spot_enabled,
            spot_url: cfg.spot_url.clone(),
            spot_key: cfg.spot_key.clone(),
            log_path: cfg.log_path.clone(),
            all_txt_path: cfg.all_txt_path.clone(),
            backend_label,
            cat_status: "Idle".to_string(),
            last_tx_summary: String::new(),
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

    fn drain_idle(
        &mut self,
        duration: Duration,
        running: &AtomicBool,
        on_chunk: &mut dyn FnMut(&[i16]),
    ) {
        match self {
            Self::Tci(t) => t.drain_idle(duration, running, on_chunk),
            Self::Audio(_) | Self::Hackrf(_) => thread::sleep(duration),
        }
    }

    fn capture_12k_streaming(
        &mut self,
        duration_sec: f64,
        running: &AtomicBool,
        on_chunk: &mut dyn FnMut(&[i16]),
    ) -> Result<(Vec<i16>, f32), String> {
        match self {
            Self::Audio(a) => a.capture_12k_streaming(duration_sec, running, on_chunk),
            Self::Hackrf(h) => h.capture_12k_streaming(duration_sec, running, on_chunk),
            Self::Tci(t) => t.capture_12k_streaming(duration_sec, running, on_chunk),
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
    let mut spec = SpectrumAnalyzer::new();

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

        // Wait until start of next UTC slot while continuously draining TCI audio
        // so the waterfall scrolls without pausing and no stale TCP backlog accumulates.
        let now_ms = utc_now_ms();
        let target_ms = (now_ms / slot_ms_len + 1) * slot_ms_len;
        let rem_ms = target_ms.saturating_sub(now_ms);
        if rem_ms > 40 && rem_ms < slot_ms_len - 150 {
            {
                let mut st = state.lock().unwrap();
                st.phase = SlotPhase::Waiting;
                st.phase_note = format!("Next {}s UTC slot in {:.1}s", slot_ms_len / 1000, rem_ms as f64 / 1000.0);
            }
            let st_wait = Arc::clone(&state);
            let mut on_wait_chunk = |chunk: &[i16]| {
                spec.push_pcm(chunk, |row, chunk_peak| {
                    let mut st = st_wait.lock().unwrap();
                    st.peak_pct = chunk_peak;
                    st.push_wf_row(row);
                });
            };
            while running.load(Ordering::Relaxed) {
                let cur_ms = utc_now_ms();
                if cur_ms >= target_ms {
                    break;
                }
                let step = (target_ms - cur_ms).min(120);
                backend.drain_idle(Duration::from_millis(step), &running, &mut on_wait_chunk);
                let interrupt = {
                    let st = state.lock().unwrap();
                    st.pending_dial_qsy.is_some() || st.pending_backend_reload.is_some()
                };
                if interrupt {
                    break;
                }
            }
            if !running.load(Ordering::Relaxed) {
                break;
            }
            let interrupt = {
                let st = state.lock().unwrap();
                st.pending_dial_qsy.is_some() || st.pending_backend_reload.is_some()
            };
            if interrupt {
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
            match backend.set_dial_hz(hz) {
                Ok(()) => {
                    let mut st = state.lock().unwrap();
                    st.engine.set_dial_hz(hz);
                    st.push_event(format!(
                        "[{}] RDV QSY -> {:.3} MHz ({})",
                        format_utc_hms(utc_sec),
                        hz as f64 / 1e6,
                        Band::from_dial_hz(hz).label()
                    ));
                }
                Err(e) => {
                    let mut st = state.lock().unwrap();
                    st.push_event(format!(
                        "[{}] RDV QSY err ({:.3} MHz): {e}",
                        format_utc_hms(utc_sec),
                        hz as f64 / 1e6
                    ));
                }
            }
            continue;
        }

        if let Some(plan) = tx_plan {
            // Apply the exact `dry_tx` snapshotted during TX planning so backend behavior,
            // UI status badge, event log, and ALL.TXT always agree for this slot.
            backend.set_dry_tx(dry_tx);
            let tones = encode_tones(plan.frame);
            {
                let mut st = state.lock().unwrap();
                st.phase = if dry_tx { SlotPhase::DryTx } else { SlotPhase::Tx };
                st.phase_note = format!("{} ({} Hz)", plan.summary, plan.audio_hz);
                st.last_tx_summary = plan.summary.clone();
                st.push_event(format!(
                    "[{}] {} {}Hz  {}",
                    format_utc_hms(utc_sec),
                    if dry_tx { "DRY-TX" } else { "TX    " },
                    plan.audio_hz,
                    plan.summary
                ));
                let mode_lbl = if mode60 { "RAD950-60" } else { "RAD950" };
                let dir = if dry_tx { "DRY-TX" } else { "TX" };
                let _ = append_all_txt(
                    &st.all_txt_path,
                    utc_sec,
                    st.engine.dial_hz,
                    mode_lbl,
                    dir,
                    None,
                    plan.audio_hz,
                    &plan.summary,
                );
            }
            // Wait nominal +400 ms into slot (cancellable immediately by Disable TX)
            let into = utc_now_ms() % slot_ms_len;
            let mut tx_ok = true;
            let mut tx_cancelled = false;
            if into < 400 && !sleep_while_tx_allowed(400 - into, &running, &state) {
                tx_ok = false;
                tx_cancelled = true;
            }

            if tx_ok {
                if mode60 {
                    for sub in 0..5u64 {
                        if !running.load(Ordering::Relaxed) || !state.lock().unwrap().tx_enabled {
                            tx_ok = false;
                            tx_cancelled = true;
                            break;
                        }
                        if sub > 0 {
                            let wait = (10_000u64.saturating_sub(utc_now_ms() % 10_000) + 400).min(1600);
                            if !sleep_while_tx_allowed(wait, &running, &state) {
                                tx_ok = false;
                                tx_cancelled = true;
                                break;
                            }
                        }
                        match transmit_burst_with_monitor(&mut backend, &tones, plan.audio_hz, &running, &state) {
                            Ok(true) => {}
                            Ok(false) => {
                                tx_ok = false;
                                tx_cancelled = true;
                                break;
                            }
                            Err(e) => {
                                state.lock().unwrap().push_event(format!("TX60 sub {sub} err: {e}"));
                                tx_ok = false;
                                break;
                            }
                        }
                    }
                } else {
                    match transmit_burst_with_monitor(&mut backend, &tones, plan.audio_hz, &running, &state) {
                        Ok(true) => {}
                        Ok(false) => {
                            tx_ok = false;
                            tx_cancelled = true;
                        }
                        Err(e) => {
                            state.lock().unwrap().push_event(format!("TX err: {e}"));
                            tx_ok = false;
                        }
                    }
                }
            }

            if tx_cancelled && running.load(Ordering::Relaxed) {
                state
                    .lock()
                    .unwrap()
                    .push_event(format!("[{}] TX cancelled (Disable TX)", format_utc_hms(utc_sec)));
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
            let cap_sec = if mode60 { 50.0 } else { 9.70 };
            spec.mark_slot_boundary();
            let st_stream = Arc::clone(&state);
            let mut on_chunk = |chunk: &[i16]| {
                spec.push_pcm(chunk, |row, chunk_peak| {
                    let mut st = st_stream.lock().unwrap();
                    st.peak_pct = chunk_peak;
                    st.push_wf_row(row);
                });
            };
            match backend.capture_12k_streaming(cap_sec, &running, &mut on_chunk) {
                Ok((pcm, peak_pct)) => {
                    if !running.load(Ordering::Relaxed) {
                        break;
                    }
                    let mut out = [Decoded::EMPTY; MAX_DECODED];
                    let n_dec = if mode60 {
                        rx60.reset();
                        for sub in 0..5usize {
                            let start = sub * 120_000;
                            let end = (start + 120_000).min(pcm.len());
                            if end > start {
                                let pcm24 = upsample_12k_to_24k_normalized(&pcm[start..end]);
                                rx60.start_subslot();
                                rx60.push_samples(&pcm24);
                            }
                        }
                        rx60.finish_minute(&mut out)
                    } else {
                        let pcm24 = upsample_12k_to_24k_normalized(&pcm);
                        decode_slot_10s_wide(&mut rx10, &pcm24, &mut out)
                    };
                    let decoded = &out[..n_dec];

                    let mut spots = Vec::new();
                    let (spot_enabled, spot_url, spot_key) = {
                        let mut st = state.lock().unwrap();
                        st.slots_rx += 1;
                        st.peak_pct = peak_pct;
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
                            let mode_lbl = if mode60 { "RAD950-60" } else { "RAD950" };
                            let _ = append_all_txt(
                                &st.all_txt_path,
                                utc_sec,
                                dial_hz,
                                mode_lbl,
                                "RX",
                                Some(d.snr_db),
                                d.audio_hz,
                                &msg_s,
                            );
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

/// Sleep up to `dur_ms` in 20 ms steps, returning `false` immediately if the app is
/// shutting down or the operator clicked Disable TX.
fn sleep_while_tx_allowed(
    dur_ms: u64,
    running: &AtomicBool,
    state: &Arc<Mutex<SharedNodeState>>,
) -> bool {
    let deadline = std::time::Instant::now() + Duration::from_millis(dur_ms);
    loop {
        if !running.load(Ordering::Relaxed) || !state.lock().unwrap().tx_enabled {
            return false;
        }
        let now = std::time::Instant::now();
        if now >= deadline {
            return true;
        }
        thread::sleep((deadline - now).min(Duration::from_millis(20)));
    }
}

/// Transmit one RAD950 burst while simultaneously feeding the real-time synthesized
/// 4-FSK TX monitor waveform into the waterfall & spectrum analyzer at ~11.7 FPS,
/// and aborting the backend immediately if the operator clicks Disable TX.
/// Returns `Ok(true)` if completed normally, `Ok(false)` if cancelled.
fn transmit_burst_with_monitor(
    backend: &mut ActiveBackend,
    tones: &[u8; urad950::N_SYMBOLS],
    audio_hz: u16,
    running: &Arc<AtomicBool>,
    state: &Arc<Mutex<SharedNodeState>>,
) -> Result<bool, String> {
    if !running.load(Ordering::Relaxed) || !state.lock().unwrap().tx_enabled {
        return Ok(false);
    }
    let active = Arc::new(AtomicBool::new(true));
    let tx_allowed = Arc::new(AtomicBool::new(true));

    let active_bg = Arc::clone(&active);
    let tx_allowed_bg = Arc::clone(&tx_allowed);
    let running_bg = Arc::clone(running);
    let st_bg = Arc::clone(state);
    let tones_copy = *tones;
    let mon = thread::spawn(move || {
        // Pick TX monitor display amplitude ~36 dB above the current RX noise floor
        // so the 4-FSK TX tones sit cleanly around 70–85% of the panafall span
        // without clipping or sidelobe bloom.
        let rx_floor_db = {
            let st = st_bg.lock().unwrap();
            let mut sorted = st.spectrum_db;
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            sorted[WF_BINS / 4].clamp(-105.0, -35.0)
        };
        let target_peak_db = (rx_floor_db + 36.0).clamp(-75.0, -10.0);
        let mon_amp = 10.0f32.powf(target_peak_db / 20.0).clamp(0.0005, 0.35);

        let tx_f32 = synth_4fsk_f32(&tones_copy, audio_hz as f64, 12_000, mon_amp);
        let tx_i16: Vec<i16> = tx_f32
            .into_iter()
            .map(|x| (x * 32767.0).clamp(-32767.0, 32767.0) as i16)
            .collect();
        let mut sa = SpectrumAnalyzer::new();
        let hop_dur = Duration::from_secs_f64(WF_HOP as f64 / 12_000.0);
        let t0 = std::time::Instant::now();
        for (idx, chunk) in tx_i16.chunks(WF_HOP).enumerate() {
            if !active_bg.load(Ordering::Relaxed) {
                break;
            }
            if !running_bg.load(Ordering::Relaxed) || !st_bg.lock().unwrap().tx_enabled {
                tx_allowed_bg.store(false, Ordering::Relaxed);
                break;
            }
            sa.push_pcm_tx(chunk, |row, _pk| {
                let mut st = st_bg.lock().unwrap();
                st.peak_pct = 75.0;
                st.push_wf_row(row);
            });
            let next_t = t0 + hop_dur * (idx as u32 + 1);
            while active_bg.load(Ordering::Relaxed) {
                if !running_bg.load(Ordering::Relaxed) || !st_bg.lock().unwrap().tx_enabled {
                    tx_allowed_bg.store(false, Ordering::Relaxed);
                    return;
                }
                let now = std::time::Instant::now();
                if now >= next_t {
                    break;
                }
                thread::sleep((next_t - now).min(Duration::from_millis(20)));
            }
        }
        // Keep watching Disable TX until backend.transmit_burst returns (e.g. TX tail)
        while active_bg.load(Ordering::Relaxed) {
            if !running_bg.load(Ordering::Relaxed) || !st_bg.lock().unwrap().tx_enabled {
                tx_allowed_bg.store(false, Ordering::Relaxed);
                return;
            }
            thread::sleep(Duration::from_millis(20));
        }
    });

    let res = backend.transmit_burst(tones, audio_hz, &tx_allowed);
    active.store(false, Ordering::Relaxed);
    let _ = mon.join();
    res?;
    Ok(tx_allowed.load(Ordering::Relaxed) && running.load(Ordering::Relaxed))
}

/// `urad950::Receiver` and `Receiver60` expect `24,000 Hz` signed 16-bit PCM.
/// Linearly upsample `12,000 Hz` capture buffers by 2x (the decoder's `LP_24K_TO_6K`
/// anti-aliasing FIR strips the 12 kHz image) and gently normalize quiet signals
/// so 16-bit fixed-point DSP retains full dynamic range.
pub fn upsample_12k_to_24k_normalized(pcm_12k: &[i16]) -> Vec<i16> {
    if pcm_12k.is_empty() {
        return Vec::new();
    }
    let peak = pcm_12k.iter().map(|&s| (s as i32).abs()).max().unwrap_or(0);
    // Boost quiet captures (e.g. TCI -35 dBFS) up toward ~16,384 peak; cap gain at 64x
    let gain: f32 = if peak > 16 && peak < 12_000 {
        (16_384.0 / peak as f32).min(64.0)
    } else {
        1.0
    };
    let mut out = Vec::with_capacity(pcm_12k.len() * 2);
    for i in 0..pcm_12k.len() {
        let a = pcm_12k[i] as f32 * gain;
        let b = if i + 1 < pcm_12k.len() {
            pcm_12k[i + 1] as f32 * gain
        } else {
            a
        };
        let mid = 0.5 * (a + b);
        out.push(a.clamp(-32767.0, 32767.0) as i16);
        out.push(mid.clamp(-32767.0, 32767.0) as i16);
    }
    out
}

/// Run `urad950::Receiver` across two overlapping timing windows (`+0 ms` and `+480 ms`)
/// so desktop `rad950` decodes signals arriving anywhere from `+0.08 s` to `+1.20 s`
/// into the UTC slot (absorbing SDR/TCI pipeline latency or ±0.5 s PC clock skew),
/// deduplicating any frame decoded in both passes.
pub fn decode_slot_10s_wide(
    rx10: &mut Receiver,
    pcm_24k: &[i16],
    out: &mut [Decoded; MAX_DECODED],
) -> usize {
    let mut total = 0usize;
    let mut pass_out = [Decoded::EMPTY; MAX_DECODED];
    for off_ms in [0usize, 480usize] {
        let start = off_ms * 24;
        if start >= pcm_24k.len() {
            continue;
        }
        rx10.reset();
        rx10.push_samples(&pcm_24k[start..]);
        let n = rx10.finish_slot(&mut pass_out);
        for d in &pass_out[..n] {
            let dup = out[..total].iter().any(|e| e.frame == d.frame);
            if !dup && total < MAX_DECODED {
                out[total] = *d;
                total += 1;
            }
        }
    }
    total
}

pub fn format_utc_ymd_hms(utc_sec: u64) -> String {
    // Howard Hinnant civil-from-days algorithm (UTC, zero dependencies)
    let days = (utc_sec / 86_400) as i64;
    let tod = utc_sec % 86_400;
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = (yoe as i64) + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let yr = if m <= 2 { y + 1 } else { y };
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        yr,
        m,
        d,
        tod / 3600,
        (tod % 3600) / 60,
        tod % 60
    )
}

#[allow(clippy::too_many_arguments)]
pub fn append_all_txt(
    path: &Path,
    utc_sec: u64,
    dial_hz: u64,
    mode_str: &str,
    dir: &str,
    snr_db: Option<i8>,
    audio_hz: u16,
    msg: &str,
) -> std::io::Result<()> {
    if path.as_os_str().is_empty() {
        return Ok(());
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            let _ = fs::create_dir_all(parent);
        }
    }
    let mut f = OpenOptions::new().create(true).append(true).open(path)?;
    let snr_col = match snr_db {
        Some(s) => format!("{s:+3} dB"),
        None => "  --- ".to_string(),
    };
    writeln!(
        f,
        "{}  {:8.3} MHz  {:6}  {:9}  {}  {:4} Hz  {}",
        format_utc_ymd_hms(utc_sec),
        dial_hz as f64 / 1e6,
        dir,
        mode_str,
        snr_col,
        audio_hz,
        msg
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audio::synth_4fsk_f32;

    #[test]
    fn append_all_txt_writes_formatted_lines() {
        let p = std::env::temp_dir().join("rad950_test_all.txt");
        let _ = fs::remove_file(&p);
        append_all_txt(&p, 1_795_223_510, 18_104_000, "RAD950", "RX", Some(4), 1043, "CQ NQ6N-1 EN61")
            .unwrap();
        append_all_txt(&p, 1_795_223_520, 18_104_000, "RAD950", "TX", None, 1400, "CQ NQ6N EN61")
            .unwrap();
        let body = fs::read_to_string(&p).unwrap();
        let _ = fs::remove_file(&p);
        let lines: Vec<&str> = body.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].contains("18.104 MHz") && lines[0].contains("RX") && lines[0].contains("+4 dB") && lines[0].contains("CQ NQ6N-1 EN61"));
        assert!(lines[1].contains("18.104 MHz") && lines[1].contains("TX") && lines[1].contains("CQ NQ6N EN61"));
    }

    #[test]
    fn tx_48k_to_12k_to_24k_decodes_cq() {
        let (call, suf, _) = parse_call_and_suffix("NQ6N-1").unwrap();
        let (grid4, _) = parse_grid("EN61").unwrap();
        let frame = Frame::Cq {
            sender: call,
            grid4,
            tx_pwr: 0,
            reserved: (suf & 0x0F) as u16,
        };
        let tones = encode_tones(frame);
        // Synthesize at 48 kHz with quiet TCI-like amplitude (0.015 = -36 dBFS)
        // and +0.85 s late arrival (beyond single-pass 0.72 s window)
        let mut tx_48k = vec![0.0f32; (48_000.0 * 0.85) as usize];
        tx_48k.extend(synth_4fsk_f32(&tones, 1050.0, 48_000, 0.015));
        tx_48k.extend(vec![0.0f32; 48_000 / 2]);

        // Decimate 4x (48 kHz -> 12 kHz) just like TciBackend
        let mut pcm_12k = Vec::with_capacity(tx_48k.len() / 4);
        for blk in tx_48k.chunks_exact(4) {
            let avg = 0.25 * (blk[0] + blk[1] + blk[2] + blk[3]);
            pcm_12k.push((avg * 32767.0).clamp(-32767.0, 32767.0) as i16);
        }

        let pcm_24k = upsample_12k_to_24k_normalized(&pcm_12k);
        let mut rx = Receiver::new();
        let mut out = [Decoded::EMPTY; MAX_DECODED];
        let n = decode_slot_10s_wide(&mut rx, &pcm_24k, &mut out);
        assert_eq!(n, 1, "expected 1 decoded RAD950 frame from late-arrival 48k->12k->24k path");
        assert_eq!(out[0].frame, frame);
    }
}
