//! Replay real conducted RT-950 Pro SI4732 12 kHz WAV captures through Layout B
//! (`CompactReceiver60`) and legacy single-phase (`SinglePhaseCompact60`) across
//! `0, +20, +40, +60 ms` timing shifts.

use std::fs::File;
use std::io::Read;
use urad950::{CompactReceiver60, Decoded, Receiver60, SinglePhaseCompact60, MAX_DECODED};

fn read_wav_12k_to_24k(path: &str) -> Vec<i16> {
    let mut f = File::open(path).expect("open wav");
    let mut buf = Vec::new();
    f.read_to_end(&mut buf).unwrap();
    // Skip 44-byte RIFF/WAVE header
    let mut s12 = Vec::with_capacity((buf.len() - 44) / 2);
    for c in buf[44..].chunks_exact(2) {
        s12.push(i16::from_le_bytes([c[0], c[1]]));
    }
    // Linear 2x upsample 12 kHz -> 24 kHz
    let mut s24 = Vec::with_capacity(s12.len() * 2);
    for i in 0..s12.len() {
        let a = s12[i] as i32;
        let b = s12[(i + 1).min(s12.len() - 1)] as i32;
        s24.push(a as i16);
        s24.push(((a + b) >> 1) as i16);
    }
    s24
}

fn decode_wav(s24: &[i16], lead_ms: usize, shift_ms: usize) -> (Vec<String>, Vec<String>, Vec<String>) {
    let mut rx_old = SinglePhaseCompact60::new();
    let mut rx_new = CompactReceiver60::new();
    let mut rx_dh = Receiver60::with_percentile_span();
    rx_dh.set_half_symbol_phase(true);
    let lead = lead_ms * 24;
    let pad = shift_ms * 24;
    let sub_len = 240_000usize;
    for sub in 0..5 {
        if sub > 0 {
            rx_old.start_subslot();
            rx_new.start_subslot();
            rx_dh.start_subslot();
        }
        let start = lead + sub * sub_len;
        let end = (start + sub_len).min(s24.len());
        let mut slot = vec![0i16; sub_len];
        if start < s24.len() {
            let take = (end - start).min(sub_len.saturating_sub(pad));
            slot[pad..pad + take].copy_from_slice(&s24[start..start + take]);
        }
        for blk in slot.chunks(192) {
            rx_old.push_samples(blk);
            rx_new.push_samples(blk);
            rx_dh.push_samples(blk);
        }
    }
    let fmt = |out: &[Decoded], n: usize| -> Vec<String> {
        out[..n]
            .iter()
            .map(|d| {
                let mut cs = [0u8; 6];
                let call = match d.frame {
                    urad950::Frame::Cq { sender, .. } => {
                        let k = sender.decode(&mut cs);
                        String::from_utf8_lossy(&cs[..k]).to_string()
                    }
                    _ => format!("{:?}", d.frame),
                };
                format!("{call}@{}Hz({:+}ms)", d.audio_hz, d.time_offset_ms)
            })
            .collect()
    };
    let mut out = [Decoded::EMPTY; MAX_DECODED];
    let n_old = rx_old.finish_minute(&mut out);
    let old_v = fmt(&out, n_old);
    let n_new = rx_new.finish_minute(&mut out);
    let new_v = fmt(&out, n_new);
    let n_dh = rx_dh.finish_minute(&mut out);
    let dh_v = fmt(&out, n_dh);
    (old_v, new_v, dh_v)
}

fn main() {
    let files: Vec<String> = std::env::args().skip(1).collect();
    println!("file                          | shift | legacy_1phase                          | layoutB_2phase                         | desk_5bit_half");
    println!("------------------------------+-------+----------------------------------------+----------------------------------------+-------------------------------");
    for path in files {
        let name = std::path::Path::new(&path)
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string();
        let s24 = read_wav_12k_to_24k(&path);
        for shift in (0usize..80).step_by(5) {
            let (old_v, new_v, dh_v) = decode_wav(&s24, 420, shift);
            println!(
                "{name:<29} | +{shift:>2}ms | {:<38} | {:<38} | {}",
                if old_v.is_empty() { "-".into() } else { old_v.join(", ") },
                if new_v.is_empty() { "-".into() } else { new_v.join(", ") },
                if dh_v.is_empty() { "-".into() } else { dh_v.join(", ") },
            );
        }
    }
}
