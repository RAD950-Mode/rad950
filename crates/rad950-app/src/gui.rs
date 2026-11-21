//! Native cross-platform desktop GUI (`eframe` / `egui`) for RAD950.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use eframe::egui;
use rad950_node::audio::list_audio_devices;
use rad950_node::config::SavedConfig;
use rad950_node::engine::{parse_call_and_suffix, parse_grid, AutoAnswer, Band, OpMode};
use rad950_node::runner::{format_utc_hms, SharedNodeState, SlotPhase};

use crate::waterfall::WaterfallWidget;

pub fn run_gui(
    state: Arc<Mutex<SharedNodeState>>,
    saved_cfg: SavedConfig,
    config_path: PathBuf,
) -> Result<(), String> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1140.0, 820.0])
            .with_min_inner_size([860.0, 620.0])
            .with_title("RAD950 — Weak-Signal 4-FSK Transceiver"),
        ..Default::default()
    };
    eframe::run_native(
        "RAD950",
        options,
        Box::new(move |cc| {
            apply_theme(&cc.egui_ctx);
            Ok(Box::new(Rad950Gui::new(state, saved_cfg, config_path)))
        }),
    )
    .map_err(|e| format!("GUI error: {e}"))
}

fn apply_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.window_fill = egui::Color32::from_rgb(18, 22, 30);
    visuals.panel_fill = egui::Color32::from_rgb(18, 22, 30);
    visuals.extreme_bg_color = egui::Color32::from_rgb(11, 14, 21);
    visuals.faint_bg_color = egui::Color32::from_rgb(24, 29, 40);
    visuals.widgets.noninteractive.bg_fill = egui::Color32::from_rgb(26, 32, 44);
    visuals.widgets.noninteractive.fg_stroke.color = egui::Color32::from_rgb(222, 228, 238);
    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(36, 44, 58);
    visuals.widgets.inactive.fg_stroke.color = egui::Color32::from_rgb(232, 238, 248);
    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(48, 59, 78);
    visuals.widgets.hovered.fg_stroke.color = egui::Color32::WHITE;
    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(56, 114, 185);
    visuals.widgets.active.fg_stroke.color = egui::Color32::WHITE;
    visuals.selection.bg_fill = egui::Color32::from_rgb(42, 102, 175);
    visuals.selection.stroke.color = egui::Color32::WHITE;
    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(10.0, 5.0);
    style.text_styles.insert(
        egui::TextStyle::Heading,
        egui::FontId::proportional(17.0),
    );
    style.text_styles.insert(
        egui::TextStyle::Body,
        egui::FontId::proportional(13.5),
    );
    style.text_styles.insert(
        egui::TextStyle::Button,
        egui::FontId::proportional(13.5),
    );
    style.text_styles.insert(
        egui::TextStyle::Monospace,
        egui::FontId::monospace(13.5),
    );
    ctx.set_style(style);
}

fn op_mode_label(m: OpMode) -> &'static str {
    match m {
        OpMode::Monitor => "Monitor (RX Only)",
        OpMode::AutoCqEven => "Run — Even Slots",
        OpMode::AutoCqOdd => "Run — Odd Slots",
        OpMode::Pounce => "Pounce (Answering)",
    }
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
    wf: WaterfallWidget,
    audio_inputs: Vec<String>,
    audio_outputs: Vec<String>,
    draft: SavedConfig,
    config_path: PathBuf,
    save_status: String,
    rig_model_str: String,
    rig_baud_str: String,
}

impl Rad950Gui {
    fn new(state: Arc<Mutex<SharedNodeState>>, saved_cfg: SavedConfig, config_path: PathBuf) -> Self {
        let (audio_inputs, audio_outputs) = list_audio_devices();
        let rig_model_str = saved_cfg.rig_model.map(|m| m.to_string()).unwrap_or_default();
        let rig_baud_str = saved_cfg.rig_baud.map(|b| b.to_string()).unwrap_or_default();
        Self {
            state,
            tab: Tab::Operate,
            wf: WaterfallWidget::new(),
            audio_inputs,
            audio_outputs,
            draft: saved_cfg,
            config_path,
            save_status: String::new(),
            rig_model_str,
            rig_baud_str,
        }
    }
}

impl eframe::App for Rad950Gui {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_millis(50));

        egui::TopBottomPanel::top("top_bar")
            .frame(
                egui::Frame::side_top_panel(&ctx.style())
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ctx, |ui| {
                let mut st = self.state.lock().unwrap();
                let now_ms = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis() as u64;
                let utc_sec = now_ms / 1000;
                let slot_ms_len = if st.engine.mode60 { 60_000u64 } else { 10_000u64 };
                let elapsed_ms = now_ms % slot_ms_len;
                let slot_num = now_ms / slot_ms_len;
                let parity_lbl = if slot_num % 2 == 0 { "EVEN" } else { "ODD" };

                // Row 1: Identity, Band selector, Status badge, UTC Clock, Navigation Tabs
                ui.horizontal_wrapped(|ui| {
                    ui.label(
                        egui::RichText::new("RAD950")
                            .size(18.0)
                            .strong()
                            .color(egui::Color32::from_rgb(110, 210, 255)),
                    );
                    ui.separator();
                    ui.label(
                        egui::RichText::new(format!("{} [{}]", st.engine.my_call_disp, st.engine.my_grid_str))
                            .strong()
                            .color(egui::Color32::WHITE),
                    );
                    ui.separator();

                    ui.label("Band:");
                    let mut cur_band = st.engine.band;
                    egui::ComboBox::from_id_salt("band_cb")
                        .width(142.0)
                        .selected_text(format!("{}  ({:.3} MHz)", cur_band.label(), st.engine.dial_hz as f64 / 1e6))
                        .show_ui(ui, |ui| {
                            for b in [Band::B10m, Band::B12m, Band::B15m, Band::B17m] {
                                let lbl = format!("{}  ({:.3} MHz)", b.label(), b.dial_hz() as f64 / 1e6);
                                if ui.selectable_value(&mut cur_band, b, lbl).clicked() && b != st.engine.band {
                                    st.pending_dial_qsy = Some(b.dial_hz());
                                }
                            }
                        });

                    ui.separator();
                    let (phase_bg, phase_txt) = match st.phase {
                        SlotPhase::Tx => (egui::Color32::from_rgb(195, 42, 42), "TX"),
                        SlotPhase::DryTx => (egui::Color32::from_rgb(185, 130, 28), "DRY-TX"),
                        SlotPhase::Rx => (egui::Color32::from_rgb(36, 138, 72), "RX"),
                        SlotPhase::Waiting => (egui::Color32::from_rgb(70, 80, 96), "WAIT"),
                    };
                    egui::Frame::new()
                        .fill(phase_bg)
                        .corner_radius(4.0)
                        .inner_margin(egui::Margin::symmetric(8, 2))
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new(phase_txt)
                                    .strong()
                                    .color(egui::Color32::WHITE),
                            );
                        });
                    ui.label(egui::RichText::new(&st.phase_note).color(egui::Color32::from_rgb(205, 215, 230)));

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.selectable_value(&mut self.tab, Tab::Settings, "Settings");
                        ui.selectable_value(&mut self.tab, Tab::Log, format!("Log ({})", st.qso_log.len()));
                        ui.selectable_value(&mut self.tab, Tab::Operate, "Operate");
                        ui.separator();
                        ui.label(
                            egui::RichText::new(format!("{} UTC", format_utc_hms(utc_sec)))
                                .monospace()
                                .strong()
                                .color(egui::Color32::from_rgb(165, 215, 255)),
                        );
                    });
                });

                ui.add_space(4.0);

                // Row 2: Standard Operating Controls + Live UTC Slot Progress Bar
                ui.horizontal_wrapped(|ui| {
                    let (tx_btn_txt, tx_fill) = if st.tx_enabled {
                        ("DISABLE TX", egui::Color32::from_rgb(200, 44, 44))
                    } else {
                        ("ENABLE TX", egui::Color32::from_rgb(34, 128, 74))
                    };
                    let tx_btn = egui::Button::new(
                        egui::RichText::new(tx_btn_txt)
                            .strong()
                            .color(egui::Color32::WHITE),
                    )
                    .fill(tx_fill)
                    .min_size(egui::vec2(108.0, 26.0));
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
                    let mut mode_sel = st.engine.mode;
                    egui::ComboBox::from_id_salt("op_mode_cb")
                        .width(142.0)
                        .selected_text(op_mode_label(mode_sel))
                        .show_ui(ui, |ui| {
                            for m in [OpMode::Monitor, OpMode::AutoCqEven, OpMode::AutoCqOdd] {
                                if ui.selectable_value(&mut mode_sel, m, op_mode_label(m)).clicked() {
                                    st.engine.set_mode(m);
                                }
                            }
                        });

                    ui.separator();
                    ui.label("Hunter:");
                    egui::ComboBox::from_id_salt("hunter_cb")
                        .width(126.0)
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

                ui.add_space(4.0);

                // Row 3: Full-width Live UTC Slot Interval Bar
                let frac = (elapsed_ms as f32) / (slot_ms_len as f32);
                let bar_col = match st.phase {
                    SlotPhase::Tx => egui::Color32::from_rgb(205, 52, 52),
                    SlotPhase::DryTx => egui::Color32::from_rgb(205, 145, 36),
                    SlotPhase::Rx => egui::Color32::from_rgb(42, 150, 86),
                    SlotPhase::Waiting => egui::Color32::from_rgb(76, 98, 130),
                };
                let bar_text = format!(
                    "{} Slot ({}):  {:4.1} / {:.0} s",
                    if st.engine.mode60 { "RAD950-60" } else { "RAD950" },
                    parity_lbl,
                    elapsed_ms as f32 / 1000.0,
                    slot_ms_len as f32 / 1000.0
                );
                let pb = egui::ProgressBar::new(frac)
                    .fill(bar_col)
                    .text(egui::RichText::new(bar_text).monospace().size(12.0).color(egui::Color32::WHITE))
                    .desired_height(18.0);
                ui.add(pb);
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

        self.wf.ui(ui, &mut st);

        ui.add_space(6.0);

        let avail = ui.available_size();
        let pane_h = (avail.y - 46.0).max(120.0);
        ui.columns(2, |cols| {
            egui::Frame::group(cols[0].style())
                .inner_margin(egui::Margin::same(8))
                .show(&mut cols[0], |ui| {
                    ui.label(
                        egui::RichText::new("Band Activity  (click any station to Answer / Pounce)")
                            .strong()
                            .size(14.5)
                            .color(egui::Color32::from_rgb(180, 215, 250)),
                    );
                    ui.separator();
                    egui::ScrollArea::vertical()
                        .id_salt("heard_scroll")
                        .max_height(pane_h)
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            let mut pounce_req = None;
                            for (idx, h) in st.heard.iter().enumerate() {
                                let line = format!(
                                    "{}  {:+3} dB  {:4} Hz  {}",
                                    h.utc_hms, h.snr_db, h.audio_hz, h.message
                                );
                                let col = if h.is_rx0_to_me {
                                    egui::Color32::from_rgb(255, 205, 85)
                                } else if h.message.starts_with("CQ ") {
                                    egui::Color32::from_rgb(115, 230, 145)
                                } else {
                                    egui::Color32::from_rgb(222, 230, 242)
                                };
                                if ui
                                    .selectable_label(
                                        false,
                                        egui::RichText::new(line).monospace().size(13.5).color(col),
                                    )
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
                });

            egui::Frame::group(cols[1].style())
                .inner_margin(egui::Margin::same(8))
                .show(&mut cols[1], |ui| {
                    ui.label(
                        egui::RichText::new("Events & Status")
                            .strong()
                            .size(14.5)
                            .color(egui::Color32::from_rgb(180, 215, 250)),
                    );
                    ui.separator();
                    egui::ScrollArea::vertical()
                        .id_salt("events_scroll")
                        .max_height(pane_h)
                        .auto_shrink([false, false])
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for ev in &st.events {
                                let col = if ev.contains("QSO LOGGED") {
                                    egui::Color32::from_rgb(95, 235, 135)
                                } else if ev.contains("err:") || ev.contains("error") {
                                    egui::Color32::from_rgb(255, 110, 110)
                                } else if ev.contains("] TX ") {
                                    egui::Color32::from_rgb(255, 150, 130)
                                } else if ev.contains("DRY-TX") {
                                    egui::Color32::from_rgb(245, 195, 85)
                                } else if ev.contains("] RX ") {
                                    egui::Color32::from_rgb(150, 220, 250)
                                } else {
                                    egui::Color32::from_rgb(210, 218, 230)
                                };
                                ui.label(egui::RichText::new(ev).monospace().size(13.5).color(col));
                            }
                        });
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
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Station, Radio, Audio & Live Spotting");
            ui.label(
                egui::RichText::new(format!("Config file: {}", self.config_path.display())).small(),
            );
            ui.separator();

            // Station
            ui.horizontal(|ui| {
                ui.label("Callsign:");
                ui.add(egui::TextEdit::singleline(&mut self.draft.call).desired_width(120.0));
                ui.label("Grid (4-char):");
                ui.add(egui::TextEdit::singleline(&mut self.draft.grid).desired_width(80.0));
            });

            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.label("Radio Backend:");
                for (val, lbl) in [
                    ("audio", "Audio + Hamlib"),
                    ("tci", "TCI (Network CAT + Audio)"),
                    ("hackrf", "Direct HackRF One"),
                ] {
                    if ui.selectable_label(self.draft.backend == val, lbl).clicked() {
                        self.draft.backend = val.to_string();
                    }
                }
            });

            ui.add_space(6.0);
            if self.draft.backend == "audio" {
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Soundcard & Hamlib CAT").strong());
                    ui.horizontal(|ui| {
                        ui.label("Audio Input:");
                        let cur_in = if self.draft.audio_in.is_empty() {
                            "(System Default)"
                        } else {
                            self.draft.audio_in.as_str()
                        };
                        egui::ComboBox::from_id_salt("audio_in_cb")
                            .selected_text(cur_in)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.draft.audio_in, String::new(), "(System Default)");
                                for d in &self.audio_inputs {
                                    ui.selectable_value(&mut self.draft.audio_in, d.clone(), d);
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Audio Output:");
                        let cur_out = if self.draft.audio_out.is_empty() {
                            "(System Default)"
                        } else {
                            self.draft.audio_out.as_str()
                        };
                        egui::ComboBox::from_id_salt("audio_out_cb")
                            .selected_text(cur_out)
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.draft.audio_out, String::new(), "(System Default)");
                                for d in &self.audio_outputs {
                                    ui.selectable_value(&mut self.draft.audio_out, d.clone(), d);
                                }
                            });
                    });
                    ui.add(egui::Slider::new(&mut self.draft.tx_audio_gain, 0.05..=0.95).text("TX Audio Gain"));
                    ui.horizontal(|ui| {
                        ui.label("Hamlib rigctld (host:port):");
                        ui.add(egui::TextEdit::singleline(&mut self.draft.rigctld_addr).desired_width(180.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Auto-spawn rigctld model:");
                        ui.add(egui::TextEdit::singleline(&mut self.rig_model_str).desired_width(70.0));
                        ui.label("Serial port:");
                        ui.add(egui::TextEdit::singleline(&mut self.draft.rig_port).desired_width(150.0));
                        ui.label("Baud:");
                        ui.add(egui::TextEdit::singleline(&mut self.rig_baud_str).desired_width(70.0));
                    });
                });
            } else if self.draft.backend == "tci" {
                ui.group(|ui| {
                    ui.label(egui::RichText::new("TCI Server (WebSocket CAT + 48 kHz Audio)").strong());
                    ui.horizontal(|ui| {
                        ui.label("TCI Server (host:port):");
                        ui.add(egui::TextEdit::singleline(&mut self.draft.tci_addr).desired_width(220.0));
                    });
                    ui.add(egui::Slider::new(&mut self.draft.tx_audio_gain, 0.05..=0.95).text("TX Audio Gain"));
                });
            } else {
                ui.group(|ui| {
                    ui.label(egui::RichText::new("Direct HackRF One").strong());
                    ui.horizontal(|ui| {
                        ui.label("Serial (optional):");
                        ui.add(egui::TextEdit::singleline(&mut self.draft.hackrf_serial).desired_width(240.0));
                    });
                    ui.horizontal(|ui| {
                        ui.add(egui::Slider::new(&mut self.draft.lna_db, 0..=40).text("RX LNA dB"));
                        ui.add(egui::Slider::new(&mut self.draft.vga_db, 0..=62).text("RX VGA dB"));
                        ui.add(egui::Slider::new(&mut self.draft.tx_vga_db, 0..=47).text("TX VGA dB"));
                    });
                });
            }

            ui.add_space(6.0);
            ui.group(|ui| {
                ui.label(egui::RichText::new("ADIF Log & Live Spotting (rad950.com)").strong());
                ui.horizontal(|ui| {
                    ui.label("ADIF Log Path:");
                    ui.add(egui::TextEdit::singleline(&mut self.draft.log_path).desired_width(360.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Activity Log (ALL.TXT):");
                    ui.add(egui::TextEdit::singleline(&mut self.draft.all_txt_path).desired_width(360.0));
                });
                ui.checkbox(&mut self.draft.spot_enabled, "Enable Live Spotting (POST /api/spots)");
                ui.horizontal(|ui| {
                    ui.label("Spots URL:");
                    ui.add(egui::TextEdit::singleline(&mut self.draft.spot_url).desired_width(280.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Bearer API Key:");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.draft.spot_key)
                            .password(true)
                            .desired_width(280.0),
                    );
                });
            });

            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button(egui::RichText::new("Save & Apply").strong()).clicked() {
                    self.draft.rig_model = self.rig_model_str.trim().parse::<u32>().ok();
                    self.draft.rig_baud = self.rig_baud_str.trim().parse::<u32>().ok();
                    {
                        let st = self.state.lock().unwrap();
                        self.draft.dry_tx = st.dry_tx;
                        self.draft.mode60 = st.engine.mode60;
                        self.draft.subch = st.engine.preferred_subch;
                        self.draft.band = st.engine.band.label().to_string();
                        self.draft.rdv_follow = st.engine.rdv_follow;
                        self.draft.rdv_call_listen = st.engine.rdv_call_listen;
                    }
                    match (
                        parse_call_and_suffix(&self.draft.call),
                        parse_grid(&self.draft.grid),
                        self.draft.to_node_config(),
                    ) {
                        (Ok((call, suf, disp)), Ok((g4, gs)), Ok(nc)) => {
                            let save_res = self.draft.save(&self.config_path);
                            let mut st = self.state.lock().unwrap();
                            st.engine.set_station(call, suf, disp, g4, gs);
                            st.spot_enabled = nc.spot_enabled;
                            st.spot_url = nc.spot_url;
                            st.spot_key = nc.spot_key;
                            st.log_path = nc.log_path;
                            st.all_txt_path = nc.all_txt_path;
                            st.pending_backend_reload = Some(nc.backend);
                            self.save_status = match save_res {
                                Ok(()) => format!("Saved to {} & reloaded backend!", self.config_path.display()),
                                Err(e) => format!("Applied live, but save failed: {e}"),
                            };
                        }
                        (Err(e), _, _) | (_, Err(e), _) | (_, _, Err(e)) => {
                            self.save_status = format!("Validation error: {e}");
                        }
                    }
                }
                if ui.button("Refresh Audio Devices").clicked() {
                    let (ins, outs) = list_audio_devices();
                    self.audio_inputs = ins;
                    self.audio_outputs = outs;
                }
                if !self.save_status.is_empty() {
                    ui.label(&self.save_status);
                }
            });
        });
    }
}
