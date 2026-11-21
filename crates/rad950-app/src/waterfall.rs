//! Real-time GPU-textured Waterfall + Spectrum Analyzer widget for RAD950.

use eframe::egui;
use rad950_node::runner::SharedNodeState;
use rad950_node::spectrum::{RowMarker, WF_BINS, WF_MAX_HZ, WF_MIN_HZ, WF_ROWS};
use urad950::SUBCHANNELS_HZ;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaterfallPalette {
    Classic,
    Turbo,
    Plasma,
    Amber,
}

impl WaterfallPalette {
    pub fn label(self) -> &'static str {
        match self {
            Self::Classic => "Classic",
            Self::Turbo => "Turbo",
            Self::Plasma => "Plasma",
            Self::Amber => "Amber",
        }
    }

    pub const ALL: [Self; 4] = [Self::Classic, Self::Turbo, Self::Plasma, Self::Amber];
}

pub struct WaterfallWidget {
    texture: Option<egui::TextureHandle>,
    last_seq: u64,
    last_floor_key: i32,
    last_span_key: i32,
    last_palette: WaterfallPalette,
    pub auto_floor: bool,
    pub floor_db: f32,
    pub span_db: f32,
    pub palette: WaterfallPalette,
    pub show_peak_hold: bool,
}

impl WaterfallWidget {
    pub fn new() -> Self {
        Self {
            texture: None,
            last_seq: u64::MAX,
            last_floor_key: i32::MIN,
            last_span_key: i32::MIN,
            last_palette: WaterfallPalette::Classic,
            auto_floor: true,
            floor_db: -85.0,
            span_db: 50.0,
            palette: WaterfallPalette::Classic,
            show_peak_hold: true,
        }
    }

    /// Render toolbar + spectrum analyzer + frequency ruler + GPU-textured waterfall.
    pub fn ui(&mut self, ui: &mut egui::Ui, st: &mut SharedNodeState) {
        // Compute auto-floor from 25th percentile of current spectrum if enabled
        let effective_floor = if self.auto_floor && !st.waterfall.is_empty() {
            let mut sorted = st.spectrum_db;
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let p25 = sorted[WF_BINS / 4];
            (p25 - 3.0).clamp(-115.0, -25.0)
        } else {
            self.floor_db
        };

        // Compact control & telemetry bar
        ui.horizontal_wrapped(|ui| {
            ui.label(
                egui::RichText::new(format!(
                    "{} ({}) | TX Subch {} ({} Hz) | RX {} / TX {} | Decodes {}",
                    st.backend_label,
                    st.cat_status,
                    st.engine.preferred_subch,
                    SUBCHANNELS_HZ[st.engine.preferred_subch],
                    st.slots_rx,
                    st.slots_tx,
                    st.frames_rx
                ))
                .small(),
            );
            ui.separator();

            // Live RX level bar
            ui.label(egui::RichText::new("RX:").small());
            let (mrect, _) = ui.allocate_exact_size(egui::vec2(68.0, 12.0), egui::Sense::hover());
            let mp = ui.painter_at(mrect);
            mp.rect_filled(mrect, 2.0, egui::Color32::from_rgb(20, 24, 32));
            let frac = (st.peak_pct / 100.0).clamp(0.0, 1.0);
            let mcol = if frac > 0.90 {
                egui::Color32::from_rgb(230, 60, 60)
            } else if frac > 0.65 {
                egui::Color32::from_rgb(235, 190, 45)
            } else {
                egui::Color32::from_rgb(55, 200, 100)
            };
            mp.rect_filled(
                egui::Rect::from_min_size(mrect.min, egui::vec2(mrect.width() * frac, mrect.height())),
                2.0,
                mcol,
            );
            ui.label(egui::RichText::new(format!("{:2.0}%", st.peak_pct)).small().monospace());

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                egui::ComboBox::from_id_salt("wf_palette_cb")
                    .width(74.0)
                    .selected_text(self.palette.label())
                    .show_ui(ui, |ui| {
                        for p in WaterfallPalette::ALL {
                            ui.selectable_value(&mut self.palette, p, p.label());
                        }
                    });
                ui.checkbox(&mut self.show_peak_hold, "Peak");
                ui.add(
                    egui::Slider::new(&mut self.span_db, 20.0..=80.0)
                        .fixed_decimals(0)
                        .suffix(" dB")
                        .show_value(true)
                        .text("Span"),
                );
                if !self.auto_floor {
                    ui.add(
                        egui::Slider::new(&mut self.floor_db, -110.0..=-30.0)
                            .fixed_decimals(0)
                            .suffix(" dB")
                            .show_value(true)
                            .text("Floor"),
                    );
                }
                ui.checkbox(&mut self.auto_floor, "Auto");
            });
        });

        // Rebuild GPU texture only when new waterfall row arrived or display mapping changed
        let floor_key = (effective_floor * 2.0).round() as i32;
        let span_key = (self.span_db * 2.0).round() as i32;
        let dirty = self.texture.is_none()
            || self.last_seq != st.wf_seq
            || self.last_floor_key != floor_key
            || self.last_span_key != span_key
            || self.last_palette != self.palette;

        if dirty {
            let img = self.build_image(st, effective_floor);
            match &mut self.texture {
                Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
                None => {
                    self.texture = Some(ui.ctx().load_texture(
                        "rad950_waterfall",
                        img,
                        egui::TextureOptions::LINEAR,
                    ));
                }
            }
            self.last_seq = st.wf_seq;
            self.last_floor_key = floor_key;
            self.last_span_key = span_key;
            self.last_palette = self.palette;
        }

        // Allocate combined canvas: Spectrum (78 px) + Ruler (18 px) + Waterfall (172 px)
        let spec_h = 78.0f32;
        let ruler_h = 18.0f32;
        let wf_h = 172.0f32;
        let total_h = spec_h + ruler_h + wf_h;
        let (full_rect, resp) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), total_h),
            egui::Sense::click_and_drag(),
        );

        let spec_rect = egui::Rect::from_min_size(full_rect.min, egui::vec2(full_rect.width(), spec_h));
        let ruler_rect = egui::Rect::from_min_size(
            egui::pos2(full_rect.left(), spec_rect.bottom()),
            egui::vec2(full_rect.width(), ruler_h),
        );
        let wf_rect = egui::Rect::from_min_size(
            egui::pos2(full_rect.left(), ruler_rect.bottom()),
            egui::vec2(full_rect.width(), wf_h),
        );

        let painter = ui.painter_at(full_rect);
        let hz_span = WF_MAX_HZ - WF_MIN_HZ;
        let hz_to_x = |hz: f32| full_rect.left() + ((hz - WF_MIN_HZ) / hz_span) * full_rect.width();

        // 1. Spectrum background & grid
        painter.rect_filled(spec_rect, 0.0, egui::Color32::from_rgb(10, 14, 22));
        for step in 1..=3 {
            let db_rel = (step as f32) * (self.span_db / 4.0);
            let y = spec_rect.bottom() - (db_rel / self.span_db) * spec_rect.height();
            painter.line_segment(
                [egui::pos2(spec_rect.left(), y), egui::pos2(spec_rect.right(), y)],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(120, 150, 190, 32)),
            );
            painter.text(
                egui::pos2(spec_rect.right() - 4.0, y - 2.0),
                egui::Align2::RIGHT_BOTTOM,
                format!("{:.0} dB", effective_floor + db_rel),
                egui::FontId::monospace(9.5),
                egui::Color32::from_rgba_unmultiplied(170, 190, 215, 140),
            );
        }

        // Subchannel passband shading on spectrum (4-FSK width ~62.5 Hz)
        let busy_mask = st.engine.occupied_subch_mask();
        for (idx, &shz) in SUBCHANNELS_HZ.iter().enumerate() {
            let x0 = hz_to_x(shz as f32 - 32.0);
            let x1 = hz_to_x(shz as f32 + 32.0);
            let selected = idx == st.engine.preferred_subch;
            let busy = (busy_mask & (1u8 << idx)) != 0;
            if selected {
                painter.rect_filled(
                    egui::Rect::from_min_max(egui::pos2(x0, spec_rect.top()), egui::pos2(x1, spec_rect.bottom())),
                    0.0,
                    egui::Color32::from_rgba_unmultiplied(235, 65, 65, 36),
                );
            } else if busy {
                painter.rect_filled(
                    egui::Rect::from_min_max(egui::pos2(x0, spec_rect.top()), egui::pos2(x1, spec_rect.bottom())),
                    0.0,
                    egui::Color32::from_rgba_unmultiplied(240, 195, 45, 28),
                );
            }
        }

        // Vertical frequency gridlines every 200 Hz
        let mut tick_hz = 400.0f32;
        while tick_hz <= 2600.0 {
            let x = hz_to_x(tick_hz);
            painter.line_segment(
                [egui::pos2(x, spec_rect.top()), egui::pos2(x, spec_rect.bottom())],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(120, 150, 190, 28)),
            );
            tick_hz += 200.0;
        }

        // Spectrum traces (peak hold + live curve + subtle under-fill)
        let db_to_y = |db: f32| {
            let norm = ((db - effective_floor) / self.span_db).clamp(0.0, 1.0);
            spec_rect.bottom() - norm * (spec_rect.height() - 4.0) - 2.0
        };

        if self.show_peak_hold {
            let pk_pts: Vec<egui::Pos2> = (0..WF_BINS)
                .map(|b| {
                    let x = full_rect.left() + (b as f32 / (WF_BINS - 1) as f32) * full_rect.width();
                    egui::pos2(x, db_to_y(st.peak_hold_db[b]))
                })
                .collect();
            painter.add(egui::Shape::line(
                pk_pts,
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 195, 70, 145)),
            ));
        }

        let live_pts: Vec<egui::Pos2> = (0..WF_BINS)
            .map(|b| {
                let x = full_rect.left() + (b as f32 / (WF_BINS - 1) as f32) * full_rect.width();
                egui::pos2(x, db_to_y(st.spectrum_db[b]))
            })
            .collect();

        // Subtle vertical fill bars under spectrum curve
        let fill_col = egui::Color32::from_rgba_unmultiplied(55, 190, 240, 38);
        for p in live_pts.iter().step_by(2) {
            painter.line_segment(
                [*p, egui::pos2(p.x, spec_rect.bottom())],
                egui::Stroke::new((full_rect.width() / (WF_BINS as f32 * 0.5)).max(1.5), fill_col),
            );
        }
        painter.add(egui::Shape::line(
            live_pts,
            egui::Stroke::new(1.5, egui::Color32::from_rgb(95, 225, 255)),
        ));

        // 2. Frequency Ruler
        painter.rect_filled(ruler_rect, 0.0, egui::Color32::from_rgb(18, 23, 34));
        painter.line_segment(
            [ruler_rect.left_top(), ruler_rect.right_top()],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(45, 58, 80)),
        );
        painter.line_segment(
            [ruler_rect.left_bottom(), ruler_rect.right_bottom()],
            egui::Stroke::new(1.0, egui::Color32::from_rgb(45, 58, 80)),
        );

        let mut f_hz = 400.0f32;
        while f_hz <= 2600.0 {
            let x = hz_to_x(f_hz);
            let is_subch = SUBCHANNELS_HZ.iter().position(|&s| (s as f32 - f_hz).abs() < 1.0);
            let (col, stroke_w) = match is_subch {
                Some(idx) if idx == st.engine.preferred_subch => (egui::Color32::from_rgb(255, 90, 90), 2.2),
                Some(idx) if (busy_mask & (1u8 << idx)) != 0 => (egui::Color32::from_rgb(245, 200, 60), 1.6),
                Some(_) => (egui::Color32::from_rgb(120, 185, 245), 1.3),
                None => (egui::Color32::from_rgb(115, 125, 145), 1.0),
            };
            painter.line_segment(
                [egui::pos2(x, ruler_rect.top()), egui::pos2(x, ruler_rect.bottom())],
                egui::Stroke::new(stroke_w, col),
            );
            painter.text(
                egui::pos2(x + 3.0, ruler_rect.center().y),
                egui::Align2::LEFT_CENTER,
                format!("{:.0}", f_hz),
                egui::FontId::monospace(10.0),
                col,
            );
            f_hz += 200.0;
        }

        // 3. GPU-Textured Waterfall
        if let Some(tex) = &self.texture {
            painter.image(
                tex.id(),
                wf_rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        } else {
            painter.rect_filled(wf_rect, 0.0, egui::Color32::from_rgb(8, 12, 24));
        }

        // Subtle TX subchannel guide lines over waterfall
        let tx_hz = SUBCHANNELS_HZ[st.engine.preferred_subch] as f32;
        for dx in [-32.0f32, 32.0f32] {
            let x = hz_to_x(tx_hz + dx);
            painter.line_segment(
                [egui::pos2(x, wf_rect.top()), egui::pos2(x, wf_rect.bottom())],
                egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 85, 85, 110)),
            );
        }

        // Outer border
        painter.rect_stroke(
            full_rect,
            2.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(55, 68, 92)),
            egui::StrokeKind::Inside,
        );

        // 4. Hover crosshair + live frequency/dBFS readout
        if let Some(pos) = resp.hover_pos() {
            if full_rect.contains(pos) {
                let frac = ((pos.x - full_rect.left()) / full_rect.width()).clamp(0.0, 1.0);
                let hz = WF_MIN_HZ + frac * hz_span;
                let bin = ((frac * (WF_BINS - 1) as f32).round() as usize).min(WF_BINS - 1);
                let db = st.spectrum_db[bin];
                let rf_mhz = (st.engine.dial_hz as f64 + hz as f64) / 1e6;
                let nearest_sub = SUBCHANNELS_HZ
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, &shz)| (shz as i32 - hz.round() as i32).abs())
                    .map(|(i, _)| i)
                    .unwrap_or(0);

                painter.line_segment(
                    [egui::pos2(pos.x, full_rect.top()), egui::pos2(pos.x, full_rect.bottom())],
                    egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 110)),
                );

                let tip = format!("{hz:.0} Hz | {rf_mhz:.6} MHz | {db:.1} dBFS | Subch {nearest_sub}");
                let tip_pos = if pos.x > full_rect.center().x {
                    egui::pos2(pos.x - 8.0, spec_rect.top() + 4.0)
                } else {
                    egui::pos2(pos.x + 8.0, spec_rect.top() + 4.0)
                };
                let align = if pos.x > full_rect.center().x {
                    egui::Align2::RIGHT_TOP
                } else {
                    egui::Align2::LEFT_TOP
                };
                painter.text(
                    tip_pos,
                    align,
                    tip,
                    egui::FontId::monospace(10.5),
                    egui::Color32::from_rgb(245, 248, 255),
                );
            }
        }

        // 5. Click to snap TX subchannel
        if resp.clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                let frac = ((pos.x - full_rect.left()) / full_rect.width()).clamp(0.0, 1.0);
                let hz = WF_MIN_HZ + frac * hz_span;
                let best = SUBCHANNELS_HZ
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, &shz)| (shz as i32 - hz.round() as i32).abs())
                    .map(|(i, _)| i)
                    .unwrap_or(4);
                st.engine.preferred_subch = best;
            }
        }
    }

    fn build_image(&self, st: &SharedNodeState, floor_db: f32) -> egui::ColorImage {
        let mut pixels = vec![egui::Color32::from_rgb(7, 10, 20); WF_BINS * WF_ROWS];
        let inv_span = 1.0 / self.span_db.max(1.0);

        for (r_idx, row) in st.waterfall.iter().take(WF_ROWS).enumerate() {
            let base = r_idx * WF_BINS;
            match row.marker {
                RowMarker::Tx => {
                    let tx_col = egui::Color32::from_rgb(95, 22, 26);
                    for x in 0..WF_BINS {
                        pixels[base + x] = tx_col;
                    }
                }
                RowMarker::SlotBoundary => {
                    for (x, &db10) in row.bins_db10.iter().enumerate() {
                        let norm = (((db10 as f32 / 10.0) - floor_db) * inv_span).clamp(0.0, 1.0);
                        let c = palette_color(self.palette, norm);
                        // Blend subtle white UTC slot divider tick
                        pixels[base + x] = egui::Color32::from_rgb(
                            c.r().saturating_add(45),
                            c.g().saturating_add(50),
                            c.b().saturating_add(60),
                        );
                    }
                }
                RowMarker::Normal => {
                    for (x, &db10) in row.bins_db10.iter().enumerate() {
                        let norm = (((db10 as f32 / 10.0) - floor_db) * inv_span).clamp(0.0, 1.0);
                        pixels[base + x] = palette_color(self.palette, norm);
                    }
                }
            }
        }

        egui::ColorImage {
            size: [WF_BINS, WF_ROWS],
            pixels,
        }
    }
}

fn palette_color(pal: WaterfallPalette, t: f32) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);
    match pal {
        WaterfallPalette::Classic => {
            // Deep navy -> royal blue -> cyan -> yellow -> bright red/white
            let stops: [(f32, [u8; 3]); 6] = [
                (0.00, [7, 10, 24]),
                (0.22, [14, 38, 118]),
                (0.45, [12, 138, 195]),
                (0.70, [140, 232, 85]),
                (0.88, [250, 200, 35]),
                (1.00, [255, 65, 45]),
            ];
            lerp_stops(&stops, t)
        }
        WaterfallPalette::Turbo => {
            let stops: [(f32, [u8; 3]); 6] = [
                (0.00, [24, 22, 68]),
                (0.20, [48, 106, 226]),
                (0.42, [34, 208, 182]),
                (0.64, [164, 242, 62]),
                (0.84, [252, 146, 36]),
                (1.00, [186, 24, 24]),
            ];
            lerp_stops(&stops, t)
        }
        WaterfallPalette::Plasma => {
            let stops: [(f32, [u8; 3]); 5] = [
                (0.00, [13, 8, 45]),
                (0.28, [106, 15, 165]),
                (0.55, [202, 52, 118]),
                (0.80, [248, 148, 48]),
                (1.00, [252, 245, 160]),
            ];
            lerp_stops(&stops, t)
        }
        WaterfallPalette::Amber => {
            let stops: [(f32, [u8; 3]); 4] = [
                (0.00, [14, 10, 6]),
                (0.35, [110, 52, 8]),
                (0.75, [235, 158, 22]),
                (1.00, [255, 244, 200]),
            ];
            lerp_stops(&stops, t)
        }
    }
}

fn lerp_stops(stops: &[(f32, [u8; 3])], t: f32) -> egui::Color32 {
    for w in stops.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        if t <= t1 {
            let f = ((t - t0) / (t1 - t0).max(1e-6)).clamp(0.0, 1.0);
            let r = (c0[0] as f32 + (c1[0] as f32 - c0[0] as f32) * f) as u8;
            let g = (c0[1] as f32 + (c1[1] as f32 - c0[1] as f32) * f) as u8;
            let b = (c0[2] as f32 + (c1[2] as f32 - c0[2] as f32) * f) as u8;
            return egui::Color32::from_rgb(r, g, b);
        }
    }
    let last = stops.last().unwrap().1;
    egui::Color32::from_rgb(last[0], last[1], last[2])
}
