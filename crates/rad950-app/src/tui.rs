//! Cross-platform terminal UI (`ratatui` + `crossterm`, works on macOS, Linux, and Windows)
//! launched when `--text` is passed on the CLI.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::execute;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen};
use rad950_node::engine::{Band, OpMode};
use rad950_node::runner::{SharedNodeState, SlotPhase};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::widgets::{Block, Borders, List, ListItem, ListState, Paragraph};
use ratatui::Terminal;
use urad950::SUBCHANNELS_HZ;

pub fn run_tui(state: Arc<Mutex<SharedNodeState>>) -> Result<(), String> {
    enable_raw_mode().map_err(|e| e.to_string())?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen).map_err(|e| e.to_string())?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend).map_err(|e| e.to_string())?;

    let res = tui_loop(&mut terminal, state);

    let _ = disable_raw_mode();
    let _ = execute!(terminal.backend_mut(), LeaveAlternateScreen);
    let _ = terminal.show_cursor();
    res
}

fn tui_loop(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    state: Arc<Mutex<SharedNodeState>>,
) -> Result<(), String> {
    let mut sel: usize = 0;

    loop {
        terminal
            .draw(|f| {
                let st = state.lock().unwrap();
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3),
                        Constraint::Min(10),
                        Constraint::Length(2),
                    ])
                    .split(f.area());

                let phase_style = match st.phase {
                    SlotPhase::Tx => Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                    SlotPhase::DryTx => Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                    SlotPhase::Rx => Style::default().fg(Color::Green),
                    SlotPhase::Waiting => Style::default().fg(Color::Gray),
                };

                let header = format!(
                    " RAD950 | {} [{}] | {:.3} MHz ({}) | Subch {} ({} Hz) | {} | TX:{} {} | 60s:{} | Hunt:{} | RDV:{} | {:?}",
                    st.engine.my_call_disp,
                    st.engine.my_grid_str,
                    st.engine.dial_hz as f64 / 1e6,
                    st.engine.band.label(),
                    st.engine.preferred_subch,
                    SUBCHANNELS_HZ[st.engine.preferred_subch],
                    st.engine.mode.label(),
                    if st.tx_enabled { "ON" } else { "OFF" },
                    if st.dry_tx { "(DRY)" } else { "(LIVE)" },
                    if st.engine.mode60 { "ON" } else { "OFF" },
                    st.engine.auto_answer.label(),
                    if st.engine.rdv_follow { "ON" } else { "OFF" },
                    st.phase
                );
                let top_title = if let Some(ref upd) = st.update_available {
                    format!("{} [Update available: {}]", st.backend_label, upd.latest_tag)
                } else {
                    st.backend_label.clone()
                };
                let top = Paragraph::new(header)
                    .style(phase_style)
                    .block(Block::default().borders(Borders::ALL).title(top_title));
                f.render_widget(top, chunks[0]);

                let body = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints([Constraint::Percentage(55), Constraint::Percentage(45)])
                    .split(chunks[1]);

                let heard_items: Vec<ListItem> = st
                    .heard
                    .iter()
                    .map(|h| {
                        ListItem::new(format!(
                            "{} {:+3}dB {:4}Hz  {}",
                            h.utc_hms, h.snr_db, h.audio_hz, h.message
                        ))
                    })
                    .collect();
                let mut list_state = ListState::default();
                if !heard_items.is_empty() {
                    list_state.select(Some(sel.min(heard_items.len() - 1)));
                }
                let heard_list = List::new(heard_items)
                    .block(Block::default().borders(Borders::ALL).title("Band Activity (Enter = Pounce)"))
                    .highlight_style(Style::default().bg(Color::Blue).fg(Color::White));
                f.render_stateful_widget(heard_list, body[0], &mut list_state);

                let ev_items: Vec<ListItem> = st
                    .events
                    .iter()
                    .rev()
                    .take(30)
                    .map(|e| ListItem::new(e.as_str()))
                    .collect();
                let ev_list = List::new(ev_items)
                    .block(Block::default().borders(Borders::ALL).title(format!(
                        "My Activity | QSOs:{} | RX:{} TX:{} | Spots:{} | Peak:{:.0}%",
                        st.qso_log.len(),
                        st.slots_rx,
                        st.slots_tx,
                        st.spots_sent,
                        st.peak_pct
                    )));
                f.render_widget(ev_list, body[1]);

                let footer = Paragraph::new(
                    " [t] TX On/Off  [d] Dry-TX  [r] Run Even/Odd/Mon  [h] Hunter  [v] Rendezvous  [6] RAD950-60  [b] Band  [s] Subch  [Enter] Pounce  [q] Quit",
                )
                .style(Style::default().fg(Color::Cyan));
                f.render_widget(footer, chunks[2]);
            })
            .map_err(|e| e.to_string())?;

        if event::poll(Duration::from_millis(120)).map_err(|e| e.to_string())? {
            if let Event::Key(k) = event::read().map_err(|e| e.to_string())? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                match k.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                    KeyCode::Up => sel = sel.saturating_sub(1),
                    KeyCode::Down => {
                        let len = state.lock().unwrap().heard.len();
                        if len > 0 && sel + 1 < len {
                            sel += 1;
                        }
                    }
                    KeyCode::Enter => {
                        let mut st = state.lock().unwrap();
                        if let Some(h) = st.heard.get(sel).cloned() {
                            if st.engine.pounce_peer(h.raw_call, h.grid, h.snr_db, h.slot, h.is_rx0_to_me) {
                                st.tx_enabled = true;
                                st.reset_watchdog();
                                st.push_event(format!("Pounce queued -> {}", h.call));
                            }
                        }
                    }
                    KeyCode::Char('t') => {
                        let mut st = state.lock().unwrap();
                        st.tx_enabled = !st.tx_enabled;
                        if st.tx_enabled {
                            st.reset_watchdog();
                        }
                    }
                    KeyCode::Char('d') => {
                        let mut st = state.lock().unwrap();
                        st.dry_tx = !st.dry_tx;
                    }
                    KeyCode::Char('r') => {
                        let mut st = state.lock().unwrap();
                        let next = match st.engine.mode {
                            OpMode::Monitor | OpMode::Pounce => OpMode::AutoCqEven,
                            OpMode::AutoCqEven => OpMode::AutoCqOdd,
                            OpMode::AutoCqOdd => OpMode::Monitor,
                        };
                        st.engine.set_mode(next);
                    }
                    KeyCode::Char('h') => {
                        let mut st = state.lock().unwrap();
                        st.engine.auto_answer = st.engine.auto_answer.next();
                    }
                    KeyCode::Char('v') => {
                        let mut st = state.lock().unwrap();
                        let on = !st.engine.rdv_follow;
                        st.engine.rdv_follow = on;
                        st.engine.rdv_call_listen = on;
                    }
                    KeyCode::Char('6') => {
                        let mut st = state.lock().unwrap();
                        let on = !st.engine.mode60;
                        st.engine.set_mode60(on);
                    }
                    KeyCode::Char('s') => {
                        let mut st = state.lock().unwrap();
                        st.engine.preferred_subch = (st.engine.preferred_subch + 1) % SUBCHANNELS_HZ.len();
                    }
                    KeyCode::Char('b') => {
                        let mut st = state.lock().unwrap();
                        let next = match st.engine.band {
                            Band::B10m => Band::B12m,
                            Band::B12m => Band::B15m,
                            Band::B15m => Band::B17m,
                            _ => Band::B10m,
                        };
                        st.pending_dial_qsy = Some(next.dial_hz());
                    }
                    _ => {}
                }
            }
        }
    }
}
