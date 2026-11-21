//! Native cross-platform desktop GUI (`eframe` / `egui`) for RAD950.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use eframe::egui;
use rad950_node::audio::list_audio_devices;
use rad950_node::engine::{AutoAnswer, Band, OpMode};
use rad950_node::runner::{SharedNodeState, SlotPhase, WF_BINS};
use urad950::SUBCHANNELS_HZ;

pub fn run_gui(state: Arc<Mutex<SharedNodeState>>) -> Result<(), String> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1080.0, 740.0])
            .with_title("RAD950 — Weak-Signal 4-FSK Transceiver"),
        ..Default::default()
    };
    eframe::run_native(
        "RAD950",
        options,
        Box::new(move |_cc| Ok(Box::new(Rad950Gui::new(state)))),
    )
    .map_err(|e| format!("GUI error: {e}"))
}

#[derive(PartialEq, Eq)]
enum Tab {
    Operate,
    Log,
    Settings,
}

struct Rad950Gui {
    state: Arc<Mutex<SharedNodeState>>,
    tab: Tab,
    audio_inputs: Vec<String>,
    audio_outputs: Vec<String>,
}

impl Rad950Gui {
    fn new(state: Arc<Mutex<SharedNodeState>>) -> Self {
        let (audio_inputs, audio_outputs) = list_audio_devices();
        Self {
            state,
            tab: Tab::Operate,
            audio_inputs,
            audio_outputs,
        }
    }
}

impl eframe::App for Rad950Gui {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_millis(120));

        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            let mut st = self.state.lock().unwrap();
            ui.horizontal_wrapped(|ui| {
                ui.heading("RAD950");
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("{} [{}]", st.engine.my_call_disp, st.engine.my_grid_str))
                        .strong(),
                );
                ui.separator();

                // Band quick-select
                for band in [Band::B10m, Band::B12m, Band::B15m, Band::B17m] {
                    let active = st.engine.band == band;
                    if ui.selectable_label(active, band.label()).clicked() && !active {
                        st.pending_dial_qsy = Some(band.dial_hz());
                    }
                }
                ui.label(format!("{:.3} MHz", st.engine.dial_hz as f64 / 1e6));
                ui.separator();

                let phase_color = match st.phase {
                    SlotPhase::Tx => egui::Color32::from_rgb(220, 60, 60),
                    SlotPhase::DryTx => egui::Color32::from_rgb(220, 160, 40),
                    SlotPhase::Rx => egui::Color32::from_rgb(60, 190, 90),
                    SlotPhase::Waiting => egui::Color32::GRAY,
                };
                ui.colored_label(phase_color, format!("{:?}: {}", st.phase, st.phase_note));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.selectable_value(&mut self.tab, Tab::Settings, "Settings");
                    ui.selectable_value(&mut self.tab, Tab::Log, format!("Log ({})", st.qso_log.len()));
                    ui.selectable_value(&mut self.tab, Tab::Operate, "Operate");
                });
            });

            ui.separator();

            ui.horizontal_wrapped(|ui| {
                let tx_btn_txt = if st.tx_enabled { "DISABLE TX" } else { "ENABLE TX" };
                let tx_btn = egui::Button::new(egui::RichText::new(tx_btn_txt).strong())
                    .fill(if st.tx_enabled {
                        egui::Color32::from_rgb(190, 40, 40)
                    } else {
                        egui::Color32::from_rgb(55, 65, 75)
                    });
                if ui.add(tx_btn).clicked() {
                    st.tx_enabled = !st.tx_enabled;
                }

                ui.checkbox(&mut st.dry_tx, "Dry TX");

                let mut m60 = st.engine.mode60;
                if ui.checkbox(&mut m60, "RAD950-60 (1-Min)").changed() {
                    st.engine.set_mode60(m60);
                }

                ui.separator();
                ui.label("Mode:");
                for (m, lbl) in [
                    (OpMode::Monitor, "Monitor"),
                    (OpMode::AutoCqEven, "Run Even"),
                    (OpMode::AutoCqOdd, "Run Odd"),
                ] {
                    if ui.selectable_label(st.engine.mode == m, lbl).clicked() {
                        st.engine.set_mode(m);
                    }
                }

                ui.separator();
                ui.label("Hunter:");
                egui::ComboBox::from_id_salt("hunter_cb")
                    .selected_text(st.engine.auto_answer.label())
                    .show_ui(ui, |ui| {
                        for aa in [
                            AutoAnswer::Off,
                            AutoAnswer::NewGrid,
                            AutoAnswer::NewBandGrid,
                            AutoAnswer::AnyNeeded,
                        ] {
                            ui.selectable_value(&mut st.engine.auto_answer, aa, aa.label());
                        }
                    });

                ui.separator();
                ui.checkbox(&mut st.engine.rdv_follow, "RDV Follow");
                ui.checkbox(&mut st.engine.rdv_call_listen, "RDV 50/50");
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::Operate => self.show_operate(ui),
            Tab::Log => self.show_log(ui),
            Tab::Settings => self.show_settings(ui),
        });
    }
}

impl Rad950Gui {
    fn show_operate(&mut self, ui: &mut egui::Ui) {
        let mut st = self.state.lock().unwrap();

        // Live Waterfall + 8 clickable subchannel markers
        ui.label(
            egui::RichText::new(format!(
                "Passband Waterfall (450–2450 Hz) — TX Subch {} ({} Hz) | RX Peak {:.0}% | RX {} / TX {} | Decodes {}",
                st.engine.preferred_subch,
                SUBCHANNELS_HZ[st.engine.preferred_subch],
                st.peak_pct,
                st.slots_rx,
                st.slots_tx,
                st.frames_rx
            ))
            .small(),
        );

        let wf_height = 140.0;
        let (rect, resp) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), wf_height),
            egui::Sense::click(),
        );
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, egui::Color32::from_rgb(8, 12, 24));

        let rows = st.waterfall.len().max(1);
        let row_h = (rect.height() - 18.0) / (rows as f32).max(24.0);
        let bin_w = rect.width() / (WF_BINS as f32);
        for (r_idx, row) in st.waterfall.iter().enumerate() {
            let y0 = rect.top() + 18.0 + (r_idx as f32) * row_h;
            if y0 >= rect.bottom() {
                break;
            }
            for (b_idx, &v) in row.iter().enumerate() {
                if v < 20 {
                    continue;
                }
                let x0 = rect.left() + (b_idx as f32) * bin_w;
                let color = waterfall_color(v);
                painter.rect_filled(
                    egui::Rect::from_min_size(egui::pos2(x0, y0), egui::vec2(bin_w + 0.5, row_h + 0.5)),
                    0.0,
                    color,
                );
            }
        }

        // Draw 8 RAD950 subchannel markers along the top strip
        let busy_mask = st.engine.occupied_subch_mask();
        for (idx, &hz) in SUBCHANNELS_HZ.iter().enumerate() {
            let frac = ((hz as f32) - 450.0) / 2000.0;
            let x = rect.left() + frac * rect.width();
            let selected = idx == st.engine.preferred_subch;
            let busy = (busy_mask & (1u8 << idx)) != 0;
            let col = if selected {
                egui::Color32::from_rgb(255, 80, 80)
            } else if busy {
                egui::Color32::from_rgb(240, 200, 50)
            } else {
                egui::Color32::from_rgb(100, 160, 220)
            };
            painter.line_segment(
                [egui::pos2(x, rect.top()), egui::pos2(x, rect.top() + 16.0)],
                egui::Stroke::new(if selected { 2.5 } else { 1.2 }, col),
            );
            painter.text(
                egui::pos2(x + 2.0, rect.top() + 2.0),
                egui::Align2::LEFT_TOP,
                format!("{hz}"),
                egui::FontId::monospace(10.0),
                col,
            );
        }
        if resp.clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                let hz = 450.0 + ((pos.x - rect.left()) / rect.width()) * 2000.0;
                let best = SUBCHANNELS_HZ
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, &shz)| (shz as i32 - hz.round() as i32).abs())
                    .map(|(i, _)| i)
                    .unwrap_or(4);
                st.engine.preferred_subch = best;
            }
        }

        ui.add_space(6.0);

        // Two-column split: Band Activity (Heard) + Activity Events
        let avail = ui.available_size();
        ui.columns(2, |cols| {
            cols[0].heading("Band Activity (click to Answer / Pounce)");
            egui::ScrollArea::vertical()
                .id_salt("heard_scroll")
                .max_height(avail.y - 30.0)
                .show(&mut cols[0], |ui| {
                    let mut pounce_req = None;
                    for (idx, h) in st.heard.iter().enumerate() {
                        let line = format!(
                            "{} {:+3}dB {:4}Hz  {}",
                            h.utc_hms, h.snr_db, h.audio_hz, h.message
                        );
                        if ui
                            .selectable_label(false, egui::RichText::new(line).monospace())
                            .clicked()
                        {
                            pounce_req = Some(idx);
                        }
                    }
                    if let Some(idx) = pounce_req {
                        let h = st.heard[idx].clone();
                        if st.engine.pounce_peer(h.raw_call, h.grid, h.snr_db, h.slot, h.is_rx0_to_me) {
                            st.tx_enabled = true;
                            st.push_event(format!("Pounce queued -> {}", h.call));
                        }
                    }
                });

            cols[1].heading("Events & Status");
            egui::ScrollArea::vertical()
                .id_salt("events_scroll")
                .max_height(avail.y - 30.0)
                .stick_to_bottom(true)
                .show(&mut cols[1], |ui| {
                    for ev in &st.events {
                        ui.label(egui::RichText::new(ev).monospace().small());
                    }
                });
        });
    }

    fn show_log(&mut self, ui: &mut egui::Ui) {
        let st = self.state.lock().unwrap();
        ui.heading(format!("ADIF QSO Log ({})", st.log_path.display()));
        ui.separator();
        egui::ScrollArea::vertical().show(ui, |ui| {
            for q in &st.qso_log {
                let mode = if q.mode60 { "RAD950-60" } else { "RAD950" };
                ui.label(
                    egui::RichText::new(format!(
                        "{}  {:>4}  {:7}  {:<6}  {:<4}  rx {:+03}  tx {:+03}",
                        rad950_node::runner::format_utc_hms(q.utc_sec),
                        q.band.label(),
                        mode,
                        q.peer,
                        q.grid,
                        q.rx_snr,
                        q.tx_snr
                    ))
                    .monospace(),
                );
            }
        });
    }

    fn show_settings(&mut self, ui: &mut egui::Ui) {
        let mut st = self.state.lock().unwrap();
        ui.heading("Radio, Audio & Live Spotting");
        ui.separator();
        ui.label(format!("Backend: {}", st.backend_label));
        ui.label(format!("CAT Status: {}", st.cat_status));
        ui.add_space(8.0);

        ui.checkbox(&mut st.spot_enabled, "Enable Live Spotting (POST /api/spots)");
        ui.horizontal(|ui| {
            ui.label("Spots URL:");
            ui.text_edit_singleline(&mut st.spot_url);
        });
        ui.horizontal(|ui| {
            ui.label("Bearer API Key:");
            ui.add(egui::TextEdit::singleline(&mut st.spot_key).password(true));
        });
        ui.label(format!("Spots sent: {} (errors: {})", st.spots_sent, st.spots_err));

        ui.add_space(10.0);
        ui.separator();
        ui.label(egui::RichText::new("Detected Audio Inputs (pass via --audio-in):").strong());
        for d in &self.audio_inputs {
            ui.label(format!("  • {d}"));
        }
        ui.label(egui::RichText::new("Detected Audio Outputs (pass via --audio-out):").strong());
        for d in &self.audio_outputs {
            ui.label(format!("  • {d}"));
        }
    }
}

fn waterfall_color(v: u8) -> egui::Color32 {
    if v < 85 {
        egui::Color32::from_rgb(0, (v as u16 * 2) as u8, 80 + v)
    } else if v < 170 {
        let t = v - 85;
        egui::Color32::from_rgb(t * 3, 170 + t, 165u8.saturating_sub(t * 2))
    } else {
        let t = v - 170;
        egui::Color32::from_rgb(255, 255u8.saturating_sub(t), t * 2)
    }
}
