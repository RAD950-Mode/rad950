//! ADIF log reader and writer for RAD950 / RAD950-60 (accepting legacy SOL950 on read).

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::Path;

use crate::engine::{Band, CompletedQso, NodeEngine};

pub fn append_adif_qso(path: &Path, qso: &CompletedQso, my_call: &str, my_grid: &str) -> std::io::Result<()> {
    let need_header = !path.exists() || fs::metadata(path).map(|m| m.len() == 0).unwrap_or(true);
    let mut f = OpenOptions::new().create(true).append(true).open(path)?;
    if need_header {
        writeln!(f, "RAD950 ADIF Log\n<adif_ver:5>3.1.4\n<programid:6>RAD950\n<eoh>")?;
    }
    let (date, time) = format_adif_utc(qso.utc_sec);
    let freq_mhz = format!("{:.6}", qso.dial_hz as f64 / 1_000_000.0);
    let band = qso.band.label();
    let mode = if qso.mode60 { "RAD950-60" } else { "RAD950" };
    let rst_sent = format!("{:+03}", qso.rx_snr);
    let rst_rcvd = format!("{:+03}", qso.tx_snr);

    write!(
        f,
        "<call:{}>{} <qso_date:8>{} <time_on:6>{} <band:{}>{} <freq:{}>{} <mode:{}>{} <rst_sent:{}>{} <rst_rcvd:{}>{}",
        qso.peer.len(), qso.peer,
        date, time,
        band.len(), band,
        freq_mhz.len(), freq_mhz,
        mode.len(), mode,
        rst_sent.len(), rst_sent,
        rst_rcvd.len(), rst_rcvd,
    )?;
    if qso.grid.len() == 4 && qso.grid != "----" {
        write!(f, " <gridsquare:4>{}", qso.grid)?;
    }
    if !my_call.is_empty() {
        write!(f, " <station_callsign:{}>{}", my_call.len(), my_call)?;
    }
    if my_grid.len() == 4 && my_grid != "----" {
        write!(f, " <my_gridsquare:4>{}", my_grid)?;
    }
    writeln!(f, " <eor>")?;
    Ok(())
}

pub fn load_adif_into_engine(path: &Path, engine: &mut NodeEngine) -> usize {
    let Ok(content) = fs::read_to_string(path) else {
        return 0;
    };
    parse_adif_worked(&content, engine)
}

pub fn parse_adif_worked(adif: &str, engine: &mut NodeEngine) -> usize {
    let bytes = adif.as_bytes();
    let n = bytes.len();
    let mut pos = 0usize;

    if let Some(eoh) = adif.to_ascii_uppercase().find("<EOH>") {
        pos = eoh + 5;
    }

    let mut loaded = 0usize;
    let mut call = String::new();
    let mut grid = String::new();
    let mut band_s = String::new();
    let mut freq_s = String::new();
    let mut mode_s = String::new();

    while pos < n {
        let Some(lt_rel) = adif[pos..].find('<') else {
            break;
        };
        let lt = pos + lt_rel;
        let Some(gt_rel) = adif[lt + 1..].find('>') else {
            break;
        };
        let gt = lt + 1 + gt_rel;
        let tag = &adif[lt + 1..gt];
        pos = gt + 1;

        let mut parts = tag.split(':');
        let name = parts.next().unwrap_or("").trim().to_ascii_uppercase();
        if name == "EOR" {
            if !call.is_empty() {
                let m = mode_s.trim().to_ascii_uppercase();
                if m.is_empty() || m == "RAD950" || m == "RAD950-60" || m == "SOL950" {
                    let band = Band::parse(&band_s)
                        .map(|(b, _)| b)
                        .or_else(|| {
                            freq_s
                                .trim()
                                .parse::<f64>()
                                .ok()
                                .map(|mhz| Band::from_dial_hz((mhz * 1_000_000.0).round() as u64))
                        })
                        .unwrap_or(Band::Other);
                    engine.record_worked(&call, &grid, band);
                    loaded += 1;
                }
            }
            call.clear();
            grid.clear();
            band_s.clear();
            freq_s.clear();
            mode_s.clear();
            continue;
        }
        let Some(len_str) = parts.next() else {
            continue;
        };
        let Ok(flen) = len_str.trim().parse::<usize>() else {
            continue;
        };
        let end = (pos + flen).min(n);
        let val = &adif[pos..end];
        pos = end;

        match name.as_str() {
            "CALL" => call = val.to_string(),
            "GRIDSQUARE" => grid = val.to_string(),
            "BAND" => band_s = val.to_string(),
            "FREQ" => freq_s = val.to_string(),
            "MODE" => mode_s = val.to_string(),
            _ => {}
        }
    }
    loaded
}

fn format_adif_utc(utc_sec: u64) -> (String, String) {
    // Civil date from Unix days (Howard Hinnant algorithm)
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
    let y = if m <= 2 { y + 1 } else { y };
    let hh = tod / 3600;
    let mm = (tod % 3600) / 60;
    let ss = tod % 60;
    (format!("{y:04}{m:02}{d:02}"), format!("{hh:02}{mm:02}{ss:02}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{parse_call_and_suffix, parse_grid};

    #[test]
    fn adif_loads_rad950_and_skips_ft8() {
        let (c, s, d) = parse_call_and_suffix("NQ6N").unwrap();
        let (g, gs) = parse_grid("EN61").unwrap();
        let mut eng = NodeEngine::new(c, s, d, g, gs, 18_104_000);
        let sample = "Header<eoh>\n\
            <call:5>K1ABC <gridsquare:4>FN42 <mode:6>RAD950 <band:3>17m <eor>\n\
            <call:5>W9FT8 <gridsquare:4>EM79 <mode:3>FT8 <band:3>17m <eor>\n\
            <call:5>W2OLD <gridsquare:4>DM13 <mode:6>SOL950 <freq:6>21.078 <eor>\n";
        assert_eq!(parse_adif_worked(sample, &mut eng), 2);
        assert!(eng.worked_grids_all.contains("FN42"));
        assert!(eng.worked_grids_all.contains("DM13"));
        assert!(!eng.worked_grids_all.contains("EM79"));
    }
}
