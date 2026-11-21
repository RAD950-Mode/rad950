//! Native cross-platform desktop GUI (`eframe` / `egui`) for RAD950.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use eframe::egui;
use rad950_node::audio::list_audio_devices;
use rad950_node::config::SavedConfig;
use rad950_node::engine::{parse_call_and_suffix, parse_grid, AutoAnswer, Band, OpMode};
use rad950_node::runner::{SharedNodeState, SlotPhase};

use crate::waterfall::WaterfallWidget;

pub fn run_gui(
    state: Arc<Mutex<SharedNodeState>>,
    saved_cfg: SavedConfig,
    config_path: PathBuf,
) -> Result<(), String> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1080.0, 760.0])
            .with_title("RAD950 — Weak-Signal 4-FSK Transceiver"),
        ..Default::default()
    };
    eframe::run_native(
        "RAD950",
        options,
        Box::new(move |_cc| Ok(Box::new(Rad950Gui::new(state, saved_cfg, config_path)))),
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

        self.wf.ui(ui, &mut st);

        ui.add_space(6.0);

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
