//! Native cross-platform desktop GUI (`eframe` / `egui`) for RAD950.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use eframe::egui;
use rad950_node::alert::{play_alert_tone, AlertTone};
use rad950_node::audio::list_audio_devices;
use rad950_node::config::{generate_station_name, SavedConfig};
use rad950_node::engine::{parse_call_and_suffix, parse_grid, AutoAnswer, Band, OpMode};
use rad950_node::runner::{format_utc_hms, trigger_full_log_sync, NeededTag, SharedNodeState, SlotPhase};

use crate::waterfall::WaterfallWidget;

pub fn run_gui(
    state: Arc<Mutex<SharedNodeState>>,
    saved_cfg: SavedConfig,
    config_path: PathBuf,
) -> Result<(), String> {
    #[cfg(windows)]
    let wgpu_options = {
        use eframe::egui_wgpu::{WgpuConfiguration, WgpuSetup, WgpuSetupCreateNew};
        use eframe::wgpu::{Backend, DeviceType};
        WgpuConfiguration {
            wgpu_setup: WgpuSetup::CreateNew(WgpuSetupCreateNew {
                native_adapter_selector: Some(Arc::new(|adapters, surface| {
                    let rank = |a: &eframe::wgpu::Adapter| -> (u8, u8) {
                        let info = a.get_info();
                        let dev_rank = match info.device_type {
                            DeviceType::DiscreteGpu => 0,
                            DeviceType::IntegratedGpu => 1,
                            DeviceType::VirtualGpu => 2,
                            DeviceType::Cpu => 3, // Microsoft Basic Render Driver (WARP)
                            DeviceType::Other => 4,
                        };
                        let backend_rank = match info.backend {
                            Backend::Dx12 => 0,
                            Backend::Vulkan => 1,
                            Backend::Gl => 2,
                            _ => 3,
                        };
                        (dev_rank, backend_rank)
                    };
                    adapters
                        .iter()
                        .filter(|a| surface.map_or(true, |s| a.is_surface_supported(s)))
                        .min_by_key(|a| rank(a))
                        .cloned()
                        .ok_or_else(|| {
                            format!(
                                "no surface-compatible wgpu adapter among {} enumerated",
                                adapters.len()
                            )
                        })
                })),
                ..Default::default()
            }),
            ..Default::default()
        }
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1140.0, 820.0])
            .with_min_inner_size([860.0, 620.0])
            .with_icon(Arc::new(build_app_icon()))
            .with_title("RAD950 — Weak-Signal 4-FSK Transceiver"),
        #[cfg(windows)]
        wgpu_options,
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

/// Generate a crisp 64×64 RGBA app icon matching the rad950.org 4-FSK ascending bars mark.
fn build_app_icon() -> egui::IconData {
    let sz = 64usize;
    let mut rgba = vec![0u8; sz * sz * 4];
    let set_px = |buf: &mut [u8], x: usize, y: usize, c: [u8; 4]| {
        if x < sz && y < sz {
            let i = (y * sz + x) * 4;
            buf[i..i + 4].copy_from_slice(&c);
        }
    };

    // Rounded dark-navy square background (#0a162c) with subtle border (#264473)
    let r = 12.0f32;
    for y in 0..sz {
        for x in 0..sz {
            let fx = x as f32 + 0.5;
            let fy = y as f32 + 0.5;
            let dx = (r - fx).max(0.0).max(fx - (sz as f32 - r));
            let dy = (r - fy).max(0.0).max(fy - (sz as f32 - r));
            let dist = (dx * dx + dy * dy).sqrt();
            if dist <= r {
                let border = dist > r - 2.0 || x < 2 || x >= sz - 2 || y < 2 || y >= sz - 2;
                let col = if border { [38, 68, 115, 255] } else { [10, 22, 44, 255] };
                set_px(&mut rgba, x, y, col);
            }
        }
    }

    // 4 ascending 4-FSK bars (matching rad950.org SVG: #737373 -> #a3a3a3 -> #d4d4d4 -> #ffffff)
    let bars: [(usize, usize, usize, [u8; 4]); 4] = [
        (11, 37, 16, [120, 140, 168, 255]),
        (22, 29, 24, [160, 182, 210, 255]),
        (33, 20, 33, [205, 222, 242, 255]),
        (44, 11, 42, [255, 255, 255, 255]),
    ];
    for (bx, by, bh, col) in bars {
        for y in by..(by + bh).min(sz - 10) {
            for x in bx..(bx + 8) {
                set_px(&mut rgba, x, y, col);
            }
        }
    }

    egui::IconData {
        rgba,
        width: sz as u32,
        height: sz as u32,
    }
}

/// Paint the rad950.org 4-FSK ascending-bars brand mark + `RAD950` wordmark.
fn paint_brand_logo(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 6.0;
        let (rect, _) = ui.allocate_exact_size(egui::vec2(20.0, 20.0), egui::Sense::hover());
        let p = ui.painter_at(rect);
        let sx = rect.width() / 24.0;
        let sy = rect.height() / 24.0;
        let bars: [(f32, f32, f32, egui::Color32); 4] = [
            (2.0, 14.0, 8.0, egui::Color32::from_rgb(115, 130, 152)),
            (7.5, 10.0, 12.0, egui::Color32::from_rgb(163, 178, 198)),
            (13.0, 6.0, 16.0, egui::Color32::from_rgb(212, 222, 236)),
            (18.5, 2.0, 20.0, egui::Color32::WHITE),
        ];
        for (x, y, h, col) in bars {
            let r = egui::Rect::from_min_size(
                egui::pos2(rect.left() + x * sx, rect.top() + y * sy),
                egui::vec2(3.6 * sx, h * sy),
            );
            p.rect_filled(r, 1.0, col);
        }

        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label(
            egui::RichText::new("RAD")
                .size(18.0)
                .color(egui::Color32::from_rgb(180, 196, 218)),
        );
        ui.label(
            egui::RichText::new("950")
                .size(18.0)
                .strong()
                .color(egui::Color32::WHITE),
        );
    });
}

fn apply_theme(ctx: &egui::Context) {
    ctx.set_theme(egui::Theme::Dark);
    ctx.options_mut(|o| o.theme_preference = egui::ThemePreference::Dark);
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Operate,
    Log,
    Settings,
}

struct Rad950Gui {
    state: Arc<Mutex<SharedNodeState>>,
    tab: Tab,
    pending_tab: Option<Tab>,
    wf: WaterfallWidget,
    audio_inputs: Vec<String>,
    audio_outputs: Vec<String>,
    draft: SavedConfig,
    applied: SavedConfig,
    config_path: PathBuf,
    save_status: String,
    rig_model_str: String,
    rig_baud_str: String,
    applied_rig_model_str: String,
    applied_rig_baud_str: String,
}

impl Rad950Gui {
    fn new(state: Arc<Mutex<SharedNodeState>>, saved_cfg: SavedConfig, config_path: PathBuf) -> Self {
        let (audio_inputs, audio_outputs) = list_audio_devices();
        let rig_model_str = saved_cfg.rig_model.map(|m| m.to_string()).unwrap_or_default();
        let rig_baud_str = saved_cfg.rig_baud.map(|b| b.to_string()).unwrap_or_default();
        Self {
            state,
            tab: Tab::Operate,
            pending_tab: None,
            wf: WaterfallWidget::new(),
            audio_inputs,
            audio_outputs,
            applied: saved_cfg.clone(),
            draft: saved_cfg,
            config_path,
            save_status: String::new(),
            applied_rig_model_str: rig_model_str.clone(),
            applied_rig_baud_str: rig_baud_str.clone(),
            rig_model_str,
            rig_baud_str,
        }
    }

    fn settings_dirty(&self) -> bool {
        self.draft != self.applied
            || self.rig_model_str != self.applied_rig_model_str
            || self.rig_baud_str != self.applied_rig_baud_str
    }

    fn discard_settings_changes(&mut self) {
        self.draft = self.applied.clone();
        self.rig_model_str = self.applied_rig_model_str.clone();
        self.rig_baud_str = self.applied_rig_baud_str.clone();
        self.save_status.clear();
    }

    fn request_tab(&mut self, target: Tab) {
        if target == self.tab {
            return;
        }
        if self.tab == Tab::Settings && self.settings_dirty() {
            self.pending_tab = Some(target);
        } else {
            self.tab = target;
        }
    }

    fn save_and_apply(&mut self) -> bool {
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
                if self.draft.web_remote_station_name.trim().is_empty() {
                    self.draft.web_remote_station_name = generate_station_name();
                }
                let save_res = self.draft.save(&self.config_path);
                let mut st = self.state.lock().unwrap();
                st.engine.set_station(call, suf, disp, g4, gs);
                st.alert_tones = nc.alert_tones;
                st.cw_id = nc.cw_id;
                st.auto_upload_qso = nc.auto_upload_qso;
                st.spot_enabled = nc.spot_enabled;
                st.spot_url = nc.spot_url;
                st.spot_key = nc.spot_key;
                st.web_remote_enabled = self.draft.web_remote_enabled;
                st.web_remote_allow_tx = self.draft.web_remote_allow_tx;
                st.web_remote_station_name = self.draft.web_remote_station_name.trim().to_string();
                st.web_remote_station_id = self.draft.web_remote_station_id.clone();
                st.log_path = nc.log_path;
                st.all_txt_path = nc.all_txt_path;
                st.tx_watchdog_secs = (nc.tx_watchdog_mins as u64) * 60;
                st.reset_watchdog();
                st.pending_backend_reload = Some(nc.backend);
                self.applied = self.draft.clone();
                self.applied_rig_model_str = self.rig_model_str.clone();
                self.applied_rig_baud_str = self.rig_baud_str.clone();
                self.save_status = match save_res {
                    Ok(()) => "Saved & reloaded radio backend.".to_string(),
                    Err(e) => format!("Applied live, but save failed: {e}"),
                };
                true
            }
            (Err(e), _, _) | (_, Err(e), _) | (_, _, Err(e)) => {
                self.save_status = format!("Error: {e}");
                false
            }
        }
    }
}

impl eframe::App for Rad950Gui {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_millis(50));

        let now_ms = rad950_node::ntp::utc_now_ms();
        let utc_sec = now_ms / 1000;

        let mut clicked_tab: Option<Tab> = None;
        let settings_dirty = self.settings_dirty();

        egui::TopBottomPanel::top("top_bar")
            .frame(
                egui::Frame::side_top_panel(&ctx.style())
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ctx, |ui| {
                let mut st = self.state.lock().unwrap();

                // Row 1: Brand Logo, Station Identity, Band selector, Navigation Tabs, UTC Clock
                ui.horizontal_wrapped(|ui| {
                    paint_brand_logo(ui);
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
                        .width(172.0)
                        .selected_text(format!("{}  ({:.3} MHz)", cur_band.label(), st.engine.dial_hz as f64 / 1e6))
                        .show_ui(ui, |ui| {
                            for b in [Band::B10m, Band::B12m, Band::B15m, Band::B17m, Band::B20m, Band::B40m] {
                                let suffix = if b.is_rt950_tx() { "" } else { " · RX on RT-950" };
                                let lbl = format!("{}  ({:.3} MHz{})", b.label(), b.dial_hz() as f64 / 1e6, suffix);
                                ui.selectable_value(&mut cur_band, b, lbl);
                            }
                        });
                    if cur_band != st.engine.band {
                        st.pending_dial_qsy = Some(cur_band.dial_hz());
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let settings_lbl = if settings_dirty {
                            "Settings *"
                        } else {
                            "Settings"
                        };
                        if ui.selectable_label(self.tab == Tab::Settings, settings_lbl).clicked() {
                            clicked_tab = Some(Tab::Settings);
                        }
                        if ui
                            .selectable_label(self.tab == Tab::Log, format!("Log ({})", st.qso_log.len()))
                            .clicked()
                        {
                            clicked_tab = Some(Tab::Log);
                        }
                        if ui.selectable_label(self.tab == Tab::Operate, "Operate").clicked() {
                            clicked_tab = Some(Tab::Operate);
                        }
                        ui.separator();
                        match rad950_node::ntp::ntp_status() {
                            Some((off_ms, rtt_ms)) if off_ms.abs() >= 250 => {
                                ui.label(
                                    egui::RichText::new(format!("NTP Δ {:+.1}s", off_ms as f64 / 1000.0))
                                        .small()
                                        .monospace()
                                        .color(egui::Color32::from_rgb(245, 195, 70)),
                                )
                                .on_hover_text(format!(
                                    "Your PC clock is off by {:+.2}s — RAD-950 is auto-correcting UTC internally (RTT {}ms)",
                                    off_ms as f64 / 1000.0,
                                    rtt_ms
                                ));
                            }
                            Some((off_ms, rtt_ms)) => {
                                ui.label(
                                    egui::RichText::new("NTP")
                                        .small()
                                        .monospace()
                                        .color(egui::Color32::from_rgb(90, 210, 125)),
                                )
                                .on_hover_text(format!(
                                    "SNTP clock synced (offset {:+}ms, RTT {}ms)",
                                    off_ms, rtt_ms
                                ));
                            }
                            None => {
                                ui.label(
                                    egui::RichText::new("Local")
                                        .small()
                                        .monospace()
                                        .color(egui::Color32::from_gray(120)),
                                )
                                .on_hover_text("Using local PC clock (SNTP sync pending or offline)");
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!("{} UTC", format_utc_hms(utc_sec)))
                                .monospace()
                                .strong()
                                .color(egui::Color32::from_rgb(165, 215, 255)),
                        );
                        if let Some(upd) = st.update_available.clone() {
                            let btn = egui::Button::new(
                                egui::RichText::new(format!("Update {}", upd.latest_tag))
                                    .small()
                                    .strong()
                                    .color(egui::Color32::from_rgb(20, 24, 30)),
                            )
                            .fill(egui::Color32::from_rgb(245, 190, 70));
                            if ui
                                .add(btn)
                                .on_hover_text(format!(
                                    "Newer RAD-950 release {} available (running v{}). Click to open download page.",
                                    upd.latest_tag, upd.current
                                ))
                                .clicked()
                            {
                                ctx.open_url(egui::OpenUrl::new_tab(&upd.release_url));
                            }
                        }
                    });
                });

                ui.add_space(4.0);

                // Row 2: Standard Operating Controls
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
                        if st.tx_enabled {
                            st.reset_watchdog();
                        }
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
                                ui.selectable_value(&mut mode_sel, m, op_mode_label(m));
                            }
                        });
                    if mode_sel != st.engine.mode {
                        st.engine.set_mode(mode_sel);
                        st.reset_watchdog();
                        st.push_event(format!("Mode -> {}", op_mode_label(mode_sel)));
                    }

                    ui.separator();
                    let is_auto_pounce = st.engine.mode == OpMode::Monitor
                        && st.engine.auto_answer == AutoAnswer::AllCqs;
                    let ap_btn = egui::Button::new(
                        egui::RichText::new(if is_auto_pounce { "Auto-Pounce ON" } else { "Auto-Pounce" })
                            .strong()
                            .color(if is_auto_pounce { egui::Color32::BLACK } else { egui::Color32::WHITE }),
                    )
                    .fill(if is_auto_pounce {
                        egui::Color32::from_rgb(56, 214, 128)
                    } else {
                        egui::Color32::from_rgb(48, 56, 72)
                    });
                    if ui
                        .add(ap_btn)
                        .on_hover_text("Listen silently (never calls CQ) and automatically reply to incoming CQs while a control operator is present")
                        .clicked()
                    {
                        if is_auto_pounce {
                            st.engine.auto_answer = AutoAnswer::Off;
                            st.push_event("Auto-Pounce OFF".to_string());
                        } else {
                            st.engine.set_mode(OpMode::Monitor);
                            st.engine.auto_answer = AutoAnswer::AllCqs;
                            st.reset_watchdog();
                            st.push_event("Auto-Pounce ON (listening for CQs)".to_string());
                        }
                    }

                    ui.separator();
                    ui.label("Hunter:");
                    egui::ComboBox::from_id_salt("hunter_cb")
                        .width(146.0)
                        .selected_text(st.engine.auto_answer.label())
                        .show_ui(ui, |ui| {
                            for aa in [
                                AutoAnswer::Off,
                                AutoAnswer::NewGrid,
                                AutoAnswer::NewBandGrid,
                                AutoAnswer::AnyNeeded,
                                AutoAnswer::AllCqs,
                            ] {
                                ui.selectable_value(&mut st.engine.auto_answer, aa, aa.label());
                            }
                        });

                    ui.separator();
                    ui.checkbox(&mut st.engine.rdv_follow, "RDV Follow");
                    ui.checkbox(&mut st.engine.rdv_call_listen, "RDV 50/50");
                    ui.separator();
                    ui.checkbox(&mut st.cw_id, "CW ID")
                        .on_hover_text("Send DE <CALL> RAD950.ORG in 22 WPM CW every 10 minutes on a CQ slot");
                });
            });

        // WSJT-X-style Bottom Status Bar (Receiving/TX badge, Mode, Decodes, Last Tx, Slot Progress Bar)
        egui::TopBottomPanel::bottom("status_bar")
            .frame(
                egui::Frame::side_top_panel(&ctx.style())
                    .fill(egui::Color32::from_rgb(24, 29, 40))
                    .inner_margin(egui::Margin::symmetric(10, 5)),
            )
            .show(ctx, |ui| {
                let mut st = self.state.lock().unwrap();
                let slot_ms_len = if st.engine.mode60 { 60_000u64 } else { 10_000u64 };
                let elapsed_ms = now_ms % slot_ms_len;
                let slot_num = now_ms / slot_ms_len;
                let parity_lbl = if slot_num % 2 == 0 { "EVEN" } else { "ODD" };

                ui.horizontal(|ui| {
                    // 1. State pill (Receiving / Transmitting / Dry TX / Waiting)
                    let (phase_bg, phase_fg, phase_txt) = match st.phase {
                        SlotPhase::Rx => (egui::Color32::from_rgb(38, 178, 72), egui::Color32::BLACK, "Receiving"),
                        SlotPhase::Tx => (egui::Color32::from_rgb(215, 48, 48), egui::Color32::WHITE, "Transmitting"),
                        SlotPhase::DryTx => (egui::Color32::from_rgb(225, 165, 36), egui::Color32::BLACK, "Dry TX"),
                        SlotPhase::Waiting => (egui::Color32::from_rgb(72, 84, 102), egui::Color32::WHITE, "Waiting"),
                    };
                    egui::Frame::new()
                        .fill(phase_bg)
                        .corner_radius(3.0)
                        .inner_margin(egui::Margin::symmetric(10, 3))
                        .show(ui, |ui| {
                            ui.label(egui::RichText::new(phase_txt).strong().color(phase_fg));
                        });

                    // 2. Mode pill (RAD950 10s = sleek navy/steel, RAD950-60 1-min = warm gold accent)
                    let (mode_lbl, mode_bg, mode_border, mode_fg) = if st.engine.mode60 {
                        (
                            "RAD950-60",
                            egui::Color32::from_rgb(56, 42, 18),
                            egui::Color32::from_rgb(212, 154, 56),
                            egui::Color32::from_rgb(255, 224, 145),
                        )
                    } else {
                        (
                            "RAD950",
                            egui::Color32::from_rgb(20, 38, 68),
                            egui::Color32::from_rgb(66, 114, 182),
                            egui::Color32::from_rgb(220, 234, 252),
                        )
                    };
                    egui::Frame::new()
                        .fill(mode_bg)
                        .stroke(egui::Stroke::new(1.0, mode_border))
                        .corner_radius(3.0)
                        .inner_margin(egui::Margin::symmetric(10, 2))
                        .show(ui, |ui| {
                            ui.label(egui::RichText::new(mode_lbl).strong().color(mode_fg));
                        });

                    // 3. Decodes / Parity pill
                    egui::Frame::new()
                        .fill(egui::Color32::from_rgb(38, 46, 62))
                        .corner_radius(3.0)
                        .inner_margin(egui::Margin::symmetric(8, 3))
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new(format!("{} | Rx {}", parity_lbl, st.frames_rx))
                                    .monospace()
                                    .color(egui::Color32::from_rgb(215, 224, 238)),
                            );
                        });

                    // 4. Last TX pill
                    let last_tx = if st.last_tx_summary.is_empty() {
                        format!("{} ({})", st.backend_label, st.cat_status)
                    } else {
                        format!("Last Tx: {}", st.last_tx_summary)
                    };
                    egui::Frame::new()
                        .fill(egui::Color32::from_rgb(32, 39, 52))
                        .corner_radius(3.0)
                        .inner_margin(egui::Margin::symmetric(8, 3))
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new(last_tx)
                                    .monospace()
                                    .color(egui::Color32::from_rgb(215, 224, 238)),
                            );
                        });

                    // 4b. Live Spotting counter pill (always visible when enabled, or after any spot sent)
                    if st.spot_enabled || st.spots_sent > 0 {
                        let (spot_txt, spot_col) = if st.spots_err > 0 {
                            (
                                format!("Spots: {} ({} err)", st.spots_sent, st.spots_err),
                                egui::Color32::from_rgb(255, 190, 75),
                            )
                        } else {
                            (
                                format!("Spots: {}", st.spots_sent),
                                egui::Color32::from_rgb(110, 225, 165),
                            )
                        };
                        egui::Frame::new()
                            .fill(egui::Color32::from_rgb(28, 38, 50))
                            .corner_radius(3.0)
                            .inner_margin(egui::Margin::symmetric(8, 3))
                            .show(ui, |ui| {
                                ui.label(egui::RichText::new(spot_txt).monospace().color(spot_col));
                            });
                    }

                    // 4c. Web Remote Control status pill
                    if st.web_remote_enabled {
                        let (rem_txt, rem_col) = if st.web_remote_connected {
                            if st.web_remote_allow_tx {
                                ("Web Remote: TX", egui::Color32::from_rgb(255, 165, 90))
                            } else {
                                ("Web Remote: RX", egui::Color32::from_rgb(100, 225, 165))
                            }
                        } else {
                            ("Web Remote: ...", egui::Color32::from_rgb(245, 195, 85))
                        };
                        egui::Frame::new()
                            .fill(egui::Color32::from_rgb(28, 38, 52))
                            .corner_radius(3.0)
                            .inner_margin(egui::Margin::symmetric(8, 3))
                            .show(ui, |ui| {
                                ui.label(egui::RichText::new(rem_txt).monospace().color(rem_col))
                                    .on_hover_text(&st.web_remote_status);
                            });
                    }

                    // 5. Right-aligned TX Watchdog (`WD:6m`) + Slot Interval Progress Bar
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let (wd_txt, wd_col) = match st.watchdog_remaining_secs() {
                            None => ("WD:Off".to_string(), egui::Color32::from_rgb(135, 145, 160)),
                            Some(rem) if rem < 60 && st.tx_enabled => (
                                format!("WD:{rem}s"),
                                egui::Color32::from_rgb(255, 185, 65),
                            ),
                            Some(rem) => (
                                format!("WD:{}m", (rem + 59) / 60),
                                egui::Color32::from_rgb(195, 210, 230),
                            ),
                        };
                        if ui
                            .add(
                                egui::Button::new(egui::RichText::new(wd_txt).monospace().color(wd_col))
                                    .fill(egui::Color32::from_rgb(34, 42, 56)),
                            )
                            .on_hover_text("TX Watchdog timer (click to reset)")
                            .clicked()
                        {
                            st.reset_watchdog();
                        }

                        let frac = (elapsed_ms as f32) / (slot_ms_len as f32);
                        let bar_col = match st.phase {
                            SlotPhase::Tx => egui::Color32::from_rgb(215, 52, 52),
                            SlotPhase::DryTx => egui::Color32::from_rgb(215, 155, 36),
                            SlotPhase::Rx => egui::Color32::from_rgb(86, 178, 240),
                            SlotPhase::Waiting => egui::Color32::from_rgb(92, 112, 142),
                        };
                        let sec_txt = format!("{}/{}", elapsed_ms / 1000, slot_ms_len / 1000);
                        ui.add(
                            egui::ProgressBar::new(frac)
                                .fill(bar_col)
                                .desired_width(240.0)
                                .desired_height(20.0)
                                .text(
                                    egui::RichText::new(sec_txt)
                                        .monospace()
                                        .strong()
                                        .color(egui::Color32::WHITE),
                                ),
                        );
                    });
                });
            });

        if let Some(t) = clicked_tab {
            self.request_tab(t);
        }

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::Operate => self.show_operate(ui),
            Tab::Log => self.show_log(ui),
            Tab::Settings => self.show_settings(ui),
        });

        if let Some(target) = self.pending_tab {
            let mut action: Option<u8> = None; // 1 = save, 2 = discard, 3 = cancel
            egui::Window::new("Unsaved Settings")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .show(ctx, |ui| {
                    ui.add_space(4.0);
                    ui.label("You have unsaved changes in Settings. Save and apply them before leaving?");
                    ui.add_space(10.0);
                    ui.horizontal(|ui| {
                        let save_btn = egui::Button::new(
                            egui::RichText::new("Save & Apply")
                                .strong()
                                .color(egui::Color32::WHITE),
                        )
                        .fill(egui::Color32::from_rgb(44, 114, 198));
                        if ui.add(save_btn).clicked() {
                            action = Some(1);
                        }
                        if ui.button("Discard Changes").clicked() {
                            action = Some(2);
                        }
                        if ui.button("Cancel").clicked() {
                            action = Some(3);
                        }
                    });
                });
            match action {
                Some(1) => {
                    self.pending_tab = None;
                    if self.save_and_apply() {
                        self.tab = target;
                    }
                }
                Some(2) => {
                    self.pending_tab = None;
                    self.discard_settings_changes();
                    self.tab = target;
                }
                Some(3) => {
                    self.pending_tab = None;
                }
                _ => {}
            }
        }
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
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("Band Activity")
                                .strong()
                                .size(14.5)
                                .color(egui::Color32::from_rgb(180, 215, 250)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            for (lbl, col) in [
                                ("Dupe", egui::Color32::from_rgb(132, 145, 165)),
                                ("New", egui::Color32::from_rgb(110, 230, 145)),
                                ("BndGrid", egui::Color32::from_rgb(80, 220, 225)),
                                ("NewGrid", egui::Color32::from_rgb(245, 115, 230)),
                                ("ToMe", egui::Color32::from_rgb(255, 195, 70)),
                            ] {
                                ui.label(egui::RichText::new(lbl).size(11.0).strong().color(col));
                            }
                        });
                    });
                    ui.separator();
                    egui::ScrollArea::vertical()
                        .id_salt("heard_scroll")
                        .max_height(pane_h)
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            let mut pounce_req = None;
                            for (idx, h) in st.heard.iter().enumerate() {
                                let (badge_bg, badge_fg, text_col) = match h.needed {
                                    NeededTag::ToMe => (
                                        egui::Color32::from_rgb(205, 65, 55),
                                        egui::Color32::WHITE,
                                        egui::Color32::from_rgb(255, 210, 95),
                                    ),
                                    NeededTag::NewGrid => (
                                        egui::Color32::from_rgb(165, 48, 155),
                                        egui::Color32::WHITE,
                                        egui::Color32::from_rgb(250, 155, 240),
                                    ),
                                    NeededTag::NewBandGrid => (
                                        egui::Color32::from_rgb(32, 130, 142),
                                        egui::Color32::WHITE,
                                        egui::Color32::from_rgb(115, 230, 235),
                                    ),
                                    NeededTag::NewCall => (
                                        egui::Color32::from_rgb(38, 128, 68),
                                        egui::Color32::WHITE,
                                        egui::Color32::from_rgb(125, 235, 155),
                                    ),
                                    NeededTag::Dupe => (
                                        egui::Color32::from_rgb(50, 58, 70),
                                        egui::Color32::from_rgb(155, 165, 180),
                                        egui::Color32::from_rgb(138, 148, 165),
                                    ),
                                    NeededTag::Info => (
                                        egui::Color32::from_rgb(44, 52, 66),
                                        egui::Color32::from_rgb(190, 200, 215),
                                        egui::Color32::from_rgb(198, 208, 222),
                                    ),
                                };

                                ui.horizontal(|ui| {
                                    egui::Frame::new()
                                        .fill(badge_bg)
                                        .corner_radius(3.0)
                                        .inner_margin(egui::Margin::symmetric(5, 1))
                                        .show(ui, |ui| {
                                            ui.label(
                                                egui::RichText::new(format!("{:^8}", h.needed.badge()))
                                                    .monospace()
                                                    .size(10.5)
                                                    .strong()
                                                    .color(badge_fg),
                                            );
                                        });
                                    let line = format!(
                                        "{}  {:+3} dB  {:4} Hz  {}",
                                        h.utc_hms, h.snr_db, h.audio_hz, h.message
                                    );
                                    if ui
                                        .selectable_label(
                                            false,
                                            egui::RichText::new(line).monospace().size(13.5).color(text_col),
                                        )
                                        .on_hover_text("Click to Answer / Pounce")
                                        .clicked()
                                    {
                                        pounce_req = Some(idx);
                                    }
                                });
                            }
                            if let Some(idx) = pounce_req {
                                let h = st.heard[idx].clone();
                                if st.engine.pounce_peer(h.raw_call, h.grid, h.snr_db, h.slot, h.is_rx0_to_me) {
                                    st.tx_enabled = true;
                                    st.reset_watchdog();
                                    st.push_event(format!("Pounce queued -> {}", h.call));
                                }
                            }
                        });
                });

            egui::Frame::group(cols[1].style())
                .inner_margin(egui::Margin::same(8))
                .show(&mut cols[1], |ui| {
                    ui.label(
                        egui::RichText::new("My Activity & QSOs")
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
        let mut sync_clicked = false;
        {
            let st = self.state.lock().unwrap();
            ui.horizontal(|ui| {
                ui.heading("QSO Log");
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("{} QSOs  •  {}", st.qso_log.len(), st.log_path.display()))
                        .color(egui::Color32::from_rgb(165, 180, 200)),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let sync_btn = egui::Button::new(
                        egui::RichText::new("Sync Log to rad950.org")
                            .strong()
                            .color(egui::Color32::WHITE),
                    )
                    .fill(egui::Color32::from_rgb(42, 108, 188));
                    if ui.add(sync_btn).clicked() {
                        sync_clicked = true;
                    }
                });
            });
            if !st.log_sync_status.is_empty() {
                let is_err = st.log_sync_status.starts_with("Error:");
                let col = if is_err {
                    egui::Color32::from_rgb(255, 115, 115)
                } else {
                    egui::Color32::from_rgb(100, 230, 145)
                };
                ui.label(egui::RichText::new(&st.log_sync_status).color(col));
            }
            ui.add_space(6.0);

            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(10))
                .show(ui, |ui| {
                    ui.label(
                        egui::RichText::new("UTC       Band  Mode       Callsign    Grid  RX SNR   TX SNR")
                            .monospace()
                            .strong()
                            .size(13.5)
                            .color(egui::Color32::from_rgb(155, 205, 245)),
                    );
                    ui.separator();
                    if st.qso_log.is_empty() {
                        ui.add_space(8.0);
                        ui.label(
                            egui::RichText::new("No QSOs logged yet. Completed 3-slot RAD950 QSOs will appear here and save to ADIF automatically.")
                                .color(egui::Color32::from_rgb(155, 165, 180)),
                        );
                    } else {
                        egui::ScrollArea::vertical()
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                for q in &st.qso_log {
                                    let mode = if q.mode60 { "RAD950-60" } else { "RAD950" };
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "{}  {:>4}  {:<9}  {:<10}  {:<4}  {:+3} dB   {:+3} dB",
                                            format_utc_hms(q.utc_sec),
                                            q.band.label(),
                                            mode,
                                            q.peer,
                                            q.grid,
                                            q.rx_snr,
                                            q.tx_snr
                                        ))
                                        .monospace()
                                        .size(13.5)
                                        .color(egui::Color32::from_rgb(225, 235, 248)),
                                    );
                                }
                            });
                    }
                });
        }
        if sync_clicked {
            trigger_full_log_sync(Arc::clone(&self.state));
        }
    }

    fn show_settings(&mut self, ui: &mut egui::Ui) {
        let section_hdr = |ui: &mut egui::Ui, title: &str, subtitle: &str| {
            ui.label(
                egui::RichText::new(title)
                    .strong()
                    .size(15.0)
                    .color(egui::Color32::from_rgb(130, 205, 255)),
            );
            ui.label(
                egui::RichText::new(subtitle)
                    .size(12.0)
                    .color(egui::Color32::from_rgb(150, 162, 180)),
            );
            ui.add_space(6.0);
        };

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.set_max_width(780.0);

            ui.horizontal(|ui| {
                ui.heading("Settings");
                ui.separator();
                ui.label(
                    egui::RichText::new(self.config_path.display().to_string())
                        .monospace()
                        .size(12.0)
                        .color(egui::Color32::from_rgb(145, 158, 178)),
                );
            });
            ui.add_space(8.0);

            // 1. Station Identity
            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(12))
                .show(ui, |ui| {
                    section_hdr(
                        ui,
                        "Station Identity",
                        "Used for CQ/RX framing, ADIF logs, and live spots.",
                    );
                    egui::Grid::new("settings_station_grid")
                        .num_columns(2)
                        .spacing([16.0, 8.0])
                        .min_col_width(170.0)
                        .show(ui, |ui| {
                            ui.label("Callsign:");
                            ui.horizontal(|ui| {
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.call)
                                        .hint_text("NQ6N or NQ6N-1")
                                        .desired_width(150.0),
                                );
                                ui.label(
                                    egui::RichText::new("Optional numeric suffix -1..-15")
                                        .size(12.0)
                                        .color(egui::Color32::from_rgb(140, 152, 170)),
                                );
                            });
                            ui.end_row();

                            ui.label("Maidenhead Grid:");
                            ui.horizontal(|ui| {
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.grid)
                                        .hint_text("EN61")
                                        .desired_width(90.0),
                                );
                                ui.label(
                                    egui::RichText::new("4-character grid square (or ----)")
                                        .size(12.0)
                                        .color(egui::Color32::from_rgb(140, 152, 170)),
                                );
                            });
                            ui.end_row();

                            ui.label("TX Watchdog:");
                            ui.horizontal(|ui| {
                                ui.add(
                                    egui::DragValue::new(&mut self.draft.tx_watchdog_mins)
                                        .range(0..=60)
                                        .suffix(" min"),
                                );
                                ui.label(
                                    egui::RichText::new("Auto-disable TX if unanswered (0 = off)")
                                        .size(12.0)
                                        .color(egui::Color32::from_rgb(140, 152, 170)),
                                );
                            });
                            ui.end_row();
                        });
                });

            ui.add_space(10.0);

            // 2. Radio & Audio Backend
            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(12))
                .show(ui, |ui| {
                    section_hdr(
                        ui,
                        "Radio & Audio Backend",
                        "Choose how RAD950 connects to your transceiver or SDR. Changes hot-reload on Save & Apply.",
                    );

                    let backend_label = |b: &str| match b {
                        "tci" => "TCI Network SDR (WebSocket CAT + Audio)",
                        "hackrf" => "Direct HackRF One (USB)",
                        _ => "Audio Soundcard + Hamlib CAT",
                    };

                    egui::Grid::new("settings_backend_grid")
                        .num_columns(2)
                        .spacing([16.0, 8.0])
                        .min_col_width(170.0)
                        .show(ui, |ui| {
                            ui.label("Backend:");
                            let mut b_sel = self.draft.backend.clone();
                            egui::ComboBox::from_id_salt("settings_backend_cb")
                                .width(310.0)
                                .selected_text(backend_label(&b_sel))
                                .show_ui(ui, |ui| {
                                    for val in ["audio", "tci", "hackrf"] {
                                        ui.selectable_value(&mut b_sel, val.to_string(), backend_label(val));
                                    }
                                });
                            if b_sel != self.draft.backend {
                                self.draft.backend = b_sel;
                            }
                            ui.end_row();

                            if self.draft.backend == "audio" {
                                ui.label("Audio Input:");
                                ui.horizontal(|ui| {
                                    let cur_in = if self.draft.audio_in.is_empty() {
                                        "(System Default)"
                                    } else {
                                        self.draft.audio_in.as_str()
                                    };
                                    egui::ComboBox::from_id_salt("audio_in_cb")
                                        .width(260.0)
                                        .selected_text(cur_in)
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(&mut self.draft.audio_in, String::new(), "(System Default)");
                                            for d in &self.audio_inputs {
                                                ui.selectable_value(&mut self.draft.audio_in, d.clone(), d);
                                            }
                                        });
                                    if ui.button("Refresh").clicked() {
                                        let (ins, outs) = list_audio_devices();
                                        self.audio_inputs = ins;
                                        self.audio_outputs = outs;
                                    }
                                });
                                ui.end_row();

                                ui.label("Audio Output:");
                                let cur_out = if self.draft.audio_out.is_empty() {
                                    "(System Default)"
                                } else {
                                    self.draft.audio_out.as_str()
                                };
                                egui::ComboBox::from_id_salt("audio_out_cb")
                                    .width(260.0)
                                    .selected_text(cur_out)
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.draft.audio_out, String::new(), "(System Default)");
                                        for d in &self.audio_outputs {
                                            ui.selectable_value(&mut self.draft.audio_out, d.clone(), d);
                                        }
                                    });
                                ui.end_row();

                                ui.label("TX Audio Level:");
                                ui.add(
                                    egui::Slider::new(&mut self.draft.tx_audio_gain, 0.05..=0.95)
                                        .fixed_decimals(2),
                                );
                                ui.end_row();

                                ui.label("Hamlib rigctld:");
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.rigctld_addr)
                                        .hint_text("127.0.0.1:4532 (blank = no CAT)")
                                        .desired_width(260.0),
                                );
                                ui.end_row();

                                ui.label("Auto-spawn rigctld:");
                                ui.horizontal(|ui| {
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.rig_model_str)
                                            .hint_text("Model #")
                                            .desired_width(72.0),
                                    );
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.draft.rig_port)
                                            .hint_text("/dev/ttyUSB0 or COM3")
                                            .desired_width(175.0),
                                    );
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.rig_baud_str)
                                            .hint_text("Baud")
                                            .desired_width(68.0),
                                    );
                                });
                                ui.end_row();
                            } else if self.draft.backend == "tci" {
                                ui.label("TCI Server (host:port):");
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.tci_addr)
                                        .hint_text("127.0.0.1:50001")
                                        .desired_width(260.0),
                                );
                                ui.end_row();

                                ui.label("TX Audio Level:");
                                ui.add(
                                    egui::Slider::new(&mut self.draft.tx_audio_gain, 0.05..=0.95)
                                        .fixed_decimals(2),
                                );
                                ui.end_row();
                            } else {
                                ui.label("Device Serial:");
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.hackrf_serial)
                                        .hint_text("Optional hex suffix (e.g. 550f)")
                                        .desired_width(260.0),
                                );
                                ui.end_row();

                                ui.label("RX LNA Gain (dB):");
                                ui.add(egui::Slider::new(&mut self.draft.lna_db, 0..=40).step_by(8.0));
                                ui.end_row();

                                ui.label("RX VGA Gain (dB):");
                                ui.add(egui::Slider::new(&mut self.draft.vga_db, 0..=62).step_by(2.0));
                                ui.end_row();

                                ui.label("TX VGA Gain (dB):");
                                ui.add(egui::Slider::new(&mut self.draft.tx_vga_db, 0..=47));
                                ui.end_row();

                                ui.label("Freq Calibration (ppm):");
                                ui.add(
                                    egui::DragValue::new(&mut self.draft.ppm)
                                        .speed(0.1)
                                        .range(-50.0..=50.0)
                                        .suffix(" ppm"),
                                );
                                ui.end_row();
                            }
                        });
                });

            ui.add_space(10.0);

            // 3. Logging & Live Spotting
            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(12))
                .show(ui, |ui| {
                    section_hdr(
                        ui,
                        "Logging & Live Spotting",
                        "Automatic ADIF QSO logging, WSJT-X-style ALL.TXT activity log, and optional rad950.org spotting.",
                    );
                    egui::Grid::new("settings_log_grid")
                        .num_columns(2)
                        .spacing([16.0, 8.0])
                        .min_col_width(170.0)
                        .show(ui, |ui| {
                            ui.label("ADIF QSO Log:");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.draft.log_path)
                                    .desired_width(440.0),
                            );
                            ui.end_row();

                            ui.label("Activity Log (ALL.TXT):");
                            ui.add(
                                egui::TextEdit::singleline(&mut self.draft.all_txt_path)
                                    .desired_width(440.0),
                            );
                            ui.end_row();

                            ui.label("Alert Tones:");
                            ui.horizontal(|ui| {
                                ui.checkbox(
                                    &mut self.draft.alert_tones,
                                    "Play chime when a new one is heard or worked",
                                );
                                ui.checkbox(&mut self.draft.cw_id, "Periodic CW ID (10m)");
                                if ui.small_button("Test Heard").clicked() {
                                    play_alert_tone(AlertTone::NewHeard);
                                }
                                if ui.small_button("Test Worked").clicked() {
                                    play_alert_tone(AlertTone::NewWorked);
                                }
                            });
                            ui.end_row();

                            ui.label("Cloud QSO Sync:");
                            ui.checkbox(
                                &mut self.draft.auto_upload_qso,
                                "Auto-upload completed QSOs to rad950.org",
                            );
                            ui.end_row();

                            ui.label("Live Spotting:");
                            ui.checkbox(&mut self.draft.spot_enabled, "Enable POST /api/spots on RX decodes");
                            ui.end_row();

                            if self.draft.spot_enabled
                                || self.draft.auto_upload_qso
                                || self.draft.web_remote_enabled
                                || !self.draft.spot_key.is_empty()
                            {
                                ui.label("rad950.org Spots URL:");
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.spot_url)
                                        .desired_width(360.0),
                                );
                                ui.end_row();

                                ui.label("rad950.org API Key:");
                                ui.horizontal(|ui| {
                                    ui.add(
                                        egui::TextEdit::singleline(&mut self.draft.spot_key)
                                            .password(true)
                                            .hint_text("r950_...")
                                            .desired_width(250.0),
                                    );
                                    if ui
                                        .button("Get API Key ↗")
                                        .on_hover_text("Open rad950.org/on-the-air/spots#api-key in your browser to generate an API key")
                                        .clicked()
                                    {
                                        ui.ctx().open_url(egui::OpenUrl::new_tab(
                                            "https://rad950.org/on-the-air/spots#api-key",
                                        ));
                                    }
                                });
                                ui.end_row();
                            }
                        });
                });

            ui.add_space(10.0);

            // 4. Web Remote Control
            egui::Frame::group(ui.style())
                .inner_margin(egui::Margin::same(12))
                .show(ui, |ui| {
                    section_hdr(
                        ui,
                        "Web Remote Control",
                        "Stream real-time panafall and decodes to your rad950.org account (rad950.org/remote) and control this station remotely.",
                    );
                    let (rem_connected, rem_status) = {
                        let st = self.state.lock().unwrap();
                        (st.web_remote_connected, st.web_remote_status.clone())
                    };
                    egui::Grid::new("settings_remote_grid")
                        .num_columns(2)
                        .spacing([16.0, 8.0])
                        .min_col_width(170.0)
                        .show(ui, |ui| {
                            ui.label("Web Remote Control:");
                            ui.checkbox(
                                &mut self.draft.web_remote_enabled,
                                "Enable remote viewing & control via rad950.org/remote",
                            );
                            ui.end_row();

                            ui.label("Remote Transmit:");
                            ui.add_enabled(
                                self.draft.web_remote_enabled,
                                egui::Checkbox::new(
                                    &mut self.draft.web_remote_allow_tx,
                                    "Allow remote TX (uncheck for monitor-only safety)",
                                ),
                            );
                            ui.end_row();

                            ui.label("Station Name:");
                            ui.horizontal(|ui| {
                                ui.add(
                                    egui::TextEdit::singleline(&mut self.draft.web_remote_station_name)
                                        .hint_text("dipole-4821")
                                        .desired_width(200.0),
                                );
                                if ui.small_button("Randomize").clicked() {
                                    self.draft.web_remote_station_name = generate_station_name();
                                }
                                ui.label(
                                    egui::RichText::new("Identifies this station when multiple rigs are online")
                                        .size(12.0)
                                        .color(egui::Color32::from_rgb(140, 152, 170)),
                                );
                            });
                            ui.end_row();

                            ui.label("Connection Status:");
                            let status_col = if rem_connected {
                                egui::Color32::from_rgb(100, 230, 145)
                            } else if self.draft.web_remote_enabled {
                                egui::Color32::from_rgb(245, 195, 85)
                            } else {
                                egui::Color32::from_rgb(145, 158, 178)
                            };
                            ui.label(egui::RichText::new(rem_status).monospace().color(status_col));
                            ui.end_row();
                        });
                });

            ui.add_space(12.0);

            // Action Bar
            ui.horizontal(|ui| {
                let save_btn = egui::Button::new(
                    egui::RichText::new("Save & Apply")
                        .strong()
                        .color(egui::Color32::WHITE),
                )
                .fill(egui::Color32::from_rgb(44, 114, 198))
                .min_size(egui::vec2(130.0, 30.0));
                if ui.add(save_btn).clicked() {
                    self.save_and_apply();
                }

                if !self.save_status.is_empty() {
                    let is_err = self.save_status.starts_with("Error:") || self.save_status.contains("failed");
                    let col = if is_err {
                        egui::Color32::from_rgb(255, 115, 115)
                    } else {
                        egui::Color32::from_rgb(100, 230, 140)
                    };
                    ui.label(egui::RichText::new(&self.save_status).color(col));
                }
            });
        });
    }
}
