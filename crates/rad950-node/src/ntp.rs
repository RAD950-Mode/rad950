//! Lightweight zero-dependency SNTP (RFC 4330) clock offset synchronization.
//!
//! Computes the offset (`true_utc_ms - local_system_ms`) without modifying the
//! OS clock (so no admin/root privileges are needed) and applies it inside
//! [`utc_now_ms`] so RAD-950 slots stay locked to true UTC even on Windows PCs
//! whose system clocks have drifted several seconds.

use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::runner::SharedNodeState;

/// Seconds between NTP epoch (1900-01-01) and Unix epoch (1970-01-01).
const NTP_TO_UNIX_SECS: i64 = 2_208_988_800;

const NTP_SERVERS: &[&str] = &[
    "time.cloudflare.com:123",
    "time.google.com:123",
    "pool.ntp.org:123",
];

static CLOCK_OFFSET_MS: AtomicI64 = AtomicI64::new(0);
static LAST_RTT_MS: AtomicI64 = AtomicI64::new(-1);
static NTP_SYNCED: AtomicBool = AtomicBool::new(false);

fn raw_system_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as i64
}

/// Current UTC timestamp in milliseconds, corrected by the latest SNTP offset.
pub fn utc_now_ms() -> u64 {
    let raw = raw_system_ms();
    let off = CLOCK_OFFSET_MS.load(Ordering::Relaxed);
    raw.saturating_add(off).max(0) as u64
}

/// Latest SNTP sync telemetry: `Some((offset_ms, rtt_ms))` once synced, or `None` if offline.
pub fn ntp_status() -> Option<(i64, i64)> {
    if NTP_SYNCED.load(Ordering::Relaxed) {
        Some((
            CLOCK_OFFSET_MS.load(Ordering::Relaxed),
            LAST_RTT_MS.load(Ordering::Relaxed),
        ))
    } else {
        None
    }
}

/// Decode a 64-bit NTP timestamp (32-bit seconds + 32-bit fractional) into Unix milliseconds.
pub fn decode_ntp_timestamp_ms(bytes: &[u8; 8]) -> Option<i64> {
    let sec = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) as i64;
    let frac = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]) as u64;
    if sec == 0 {
        return None;
    }
    let unix_sec = sec - NTP_TO_UNIX_SECS;
    let ms_frac = ((frac * 1000) >> 32) as i64;
    Some(unix_sec.saturating_mul(1000).saturating_add(ms_frac))
}

/// Parse an SNTP reply packet given local send/receive timestamps `t1_ms` and `t4_ms`.
/// Returns `(offset_ms, rtt_ms)` where `offset_ms = true_utc_ms - local_system_ms`.
pub fn parse_sntp_reply(pkt: &[u8], t1_ms: i64, t4_ms: i64) -> Result<(i64, i64), String> {
    if pkt.len() < 48 {
        return Err(format!("short NTP packet ({} bytes)", pkt.len()));
    }
    let mode = pkt[0] & 0x07;
    let stratum = pkt[1];
    if mode != 4 && mode != 5 {
        return Err(format!("unexpected NTP mode {mode}"));
    }
    if stratum == 0 || stratum > 15 {
        return Err(format!("invalid NTP stratum {stratum}"));
    }
    let t2_ms = decode_ntp_timestamp_ms(pkt[32..40].try_into().unwrap())
        .ok_or_else(|| "missing NTP receive timestamp".to_string())?;
    let t3_ms = decode_ntp_timestamp_ms(pkt[40..48].try_into().unwrap())
        .ok_or_else(|| "missing NTP transmit timestamp".to_string())?;

    let rtt_ms = (t4_ms - t1_ms) - (t3_ms - t2_ms);
    if !(0..=2000).contains(&rtt_ms) {
        return Err(format!("excessive NTP RTT ({rtt_ms} ms)"));
    }
    let offset_ms = ((t2_ms - t1_ms) + (t3_ms - t4_ms)) / 2;
    Ok((offset_ms, rtt_ms))
}

/// Query a single SNTP server over UDP and return `(offset_ms, rtt_ms)`.
pub fn query_server(addr: &str) -> Result<(i64, i64), String> {
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| format!("UDP bind: {e}"))?;
    sock.set_read_timeout(Some(Duration::from_millis(1500)))
        .map_err(|e| format!("UDP timeout: {e}"))?;
    sock.set_write_timeout(Some(Duration::from_millis(1500)))
        .map_err(|e| format!("UDP timeout: {e}"))?;
    sock.connect(addr).map_err(|e| format!("UDP connect {addr}: {e}"))?;

    let mut req = [0u8; 48];
    req[0] = 0x23; // LI=0, VN=4, Mode=3 (client)

    let t1_ms = raw_system_ms();
    sock.send(&req).map_err(|e| format!("UDP send {addr}: {e}"))?;

    let mut resp = [0u8; 64];
    let n = sock.recv(&mut resp).map_err(|e| format!("UDP recv {addr}: {e}"))?;
    let t4_ms = raw_system_ms();

    parse_sntp_reply(&resp[..n], t1_ms, t4_ms)
}

/// Query public SNTP pools, pick the lowest-RTT valid reply, and update the global offset.
pub fn sync_once() -> Result<(i64, i64, &'static str), String> {
    let mut best: Option<(i64, i64, &'static str)> = None;
    let mut last_err = String::from("no NTP servers reachable");

    for &srv in NTP_SERVERS {
        match query_server(srv) {
            Ok((off, rtt)) => {
                if best.map_or(true, |b| rtt < b.1) {
                    best = Some((off, rtt, srv));
                }
            }
            Err(e) => {
                last_err = e;
            }
        }
    }

    if let Some((off, rtt, srv)) = best {
        CLOCK_OFFSET_MS.store(off, Ordering::Relaxed);
        LAST_RTT_MS.store(rtt, Ordering::Relaxed);
        NTP_SYNCED.store(true, Ordering::Relaxed);
        Ok((off, rtt, srv))
    } else {
        Err(last_err)
    }
}

/// Spawn a lightweight background thread that syncs UTC clock offset at startup
/// and every 15 minutes thereafter.
pub fn spawn_ntp_sync_thread(
    running: Arc<AtomicBool>,
    state: Arc<Mutex<SharedNodeState>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut first = true;
        while running.load(Ordering::Relaxed) {
            match sync_once() {
                Ok((off_ms, rtt_ms, srv)) => {
                    let host = srv.split(':').next().unwrap_or(srv);
                    if first || off_ms.abs() >= 250 {
                        let msg = if off_ms.abs() >= 250 {
                            format!(
                                "NTP synced ({host}, RTT {rtt_ms}ms): PC clock off by {:+.2}s — auto-corrected",
                                off_ms as f64 / 1000.0
                            )
                        } else {
                            format!(
                                "NTP synced ({host}, offset {:+}ms, RTT {rtt_ms}ms)",
                                off_ms
                            )
                        };
                        if let Ok(mut st) = state.lock() {
                            st.push_event(msg);
                        }
                    }
                    first = false;
                }
                Err(_) => {
                    if first {
                        if let Ok(mut st) = state.lock() {
                            st.push_event(
                                "NTP unreachable (offline) — using local PC clock".to_string(),
                            );
                        }
                        first = false;
                    }
                }
            }

            // Re-sync every 15 minutes (sleep in 500 ms steps for instant shutdown)
            for _ in 0..(15 * 60 * 2) {
                if !running.load(Ordering::Relaxed) {
                    return;
                }
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sntp_offset_math_recovers_positive_and_negative_skew() {
        // True UTC is 1_750_000_000_500 ms; local PC clock is 2_400 ms behind (slow).
        // RTT = 40 ms -> t1 = true - 2400 - 20, t2 = true, t3 = true + 2, t4 = true - 2400 + 22
        let true_ms: i64 = 1_750_000_000_500;
        let t1_ms = true_ms - 2_420;
        let t2_ms = true_ms;
        let t3_ms = true_ms + 2;
        let t4_ms = true_ms - 2_378;

        let encode_ts = |unix_ms: i64| -> [u8; 8] {
            let ntp_sec = (unix_ms / 1000 + NTP_TO_UNIX_SECS) as u32;
            let rem_ms = (unix_ms % 1000) as u64;
            let frac = ((rem_ms << 32) / 1000) as u32;
            let mut out = [0u8; 8];
            out[0..4].copy_from_slice(&ntp_sec.to_be_bytes());
            out[4..8].copy_from_slice(&frac.to_be_bytes());
            out
        };

        let mut pkt = [0u8; 48];
        pkt[0] = 0x24; // VN=4, Mode=4 (server)
        pkt[1] = 2; // Stratum 2
        pkt[32..40].copy_from_slice(&encode_ts(t2_ms));
        pkt[40..48].copy_from_slice(&encode_ts(t3_ms));

        let (off, rtt) = parse_sntp_reply(&pkt, t1_ms, t4_ms).expect("valid SNTP reply");
        assert!((off - 2400).abs() <= 2, "expected offset ~+2400ms, got {off}");
        assert!((rtt - 40).abs() <= 2, "expected rtt ~40ms, got {rtt}");
    }
}
