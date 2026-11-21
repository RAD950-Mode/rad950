//! Decorated station identity (`[PREFIX/]BASE[-SUFFIX]`) wire extension (`docs/rad950-firmware-feature.md` §10).
//!
//! - Uses `type = 1` (bit 63 = `1`) and reserved marker `0b111111` in bits `62..57` (`s_raw >= 0x0FC0_0000 > Callsign::MAX_RAW`),
//!   so legacy decoders strictly reject extension fragments rather than misparsing them as `CQ` or `RX`.
//! - Packs `[PREFIX/]BASE[-SUFFIX]` into 53 bits: `base` (28 b) + `prefix` (21 b, up to 4 base-37 chars) + `suffix` (4 b, `0..=15`).
//! - `ExtMsg::Cq` spans `3` fragments (`135` payload bits); `ExtMsg::Rx` spans `4` fragments (`180` payload bits),
//!   each protected by a 16-bit CCITT CRC over the full reassembled logical payload in addition to each fragment's CRC-14 + LDPC.

use crate::frame::{decode_snr_db, encode_snr_db, CallError, Callsign, Frame, GRID_MAX};

/// Extension marker placed in bits `62..57` (`0b111111`) with bit 63 = `1`.
pub const EXT_MARKER_BITS: u64 = 0x3F;
/// Payload bits carried per 64-bit extension fragment (`bits 44..0`).
pub const FRAG_BITS: usize = 45;
pub const FRAG_MASK: u64 = (1u64 << FRAG_BITS) - 1;

pub const EXT_KIND_CQ: u8 = 0;
pub const EXT_KIND_RX: u8 = 1;
pub const EXT_KIND_RX_ACK: u8 = 2;

/// Number of fragments per logical extension message.
pub const CQ_FRAGS: usize = 3;
pub const RX_FRAGS: usize = 4;

const PREFIX_MAX_RAW: u32 = 37 * 37 * 37 * 37 - 1; // 1_874_160 < 2^21

fn enc_ch37(c: u8) -> Option<u32> {
    match c.to_ascii_uppercase() {
        b' ' => Some(0),
        b'A'..=b'Z' => Some((c.to_ascii_uppercase() - b'A') as u32 + 1),
        b'0'..=b'9' => Some((c - b'0') as u32 + 27),
        _ => None,
    }
}

fn dec_ch37(v: u32) -> u8 {
    match v {
        0 => b' ',
        1..=26 => b'A' + (v as u8 - 1),
        27..=36 => b'0' + (v as u8 - 27),
        _ => b' ',
    }
}

/// 53-bit decorated station identity: optional `1..=4` char alphanumeric prefix, 28-bit base [`Callsign`],
/// and optional `1..=15` numeric suffix (`0` = none).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct StationId {
    pub base: Callsign,
    /// Packed 4 base-37 characters (`0` = no prefix).
    pub prefix_raw: u32,
    /// `0` = no suffix, `1..=15` = `-1`..`-15`.
    pub suffix: u8,
}

impl StationId {
    pub const fn plain(base: Callsign) -> Self {
        Self { base, prefix_raw: 0, suffix: 0 }
    }

    pub const fn is_decorated(self) -> bool {
        self.prefix_raw != 0 || self.suffix != 0
    }

    /// Parse `[PREFIX/]BASE[-SUFFIX]` (e.g. `NQ6N`, `NQ6N-1`, `KP2/NQ6N`, `KP2/NQ6N-15`).
    pub fn parse(s: &[u8]) -> Result<Self, CallError> {
        let s = trim_ascii(s);
        if s.is_empty() {
            return Err(CallError::Empty);
        }
        let (pfx, rest) = if let Some(slash) = s.iter().position(|&b| b == b'/') {
            let p = &s[..slash];
            let r = &s[slash + 1..];
            if p.is_empty() || p.len() > 4 || r.is_empty() {
                return Err(CallError::UnsupportedFormat);
            }
            (Some(p), r)
        } else {
            (None, s)
        };
        let (base_s, suffix) = if let Some(dash) = rest.iter().position(|&b| b == b'-') {
            let b_part = &rest[..dash];
            let suf_part = &rest[dash + 1..];
            if suf_part.is_empty() || suf_part.len() > 2 || !suf_part.iter().all(u8::is_ascii_digit) {
                return Err(CallError::UnsupportedFormat);
            }
            let mut v = 0u8;
            for &d in suf_part {
                v = v.saturating_mul(10).saturating_add(d - b'0');
            }
            if !(1..=15).contains(&v) {
                return Err(CallError::UnsupportedFormat);
            }
            (b_part, v)
        } else {
            (rest, 0u8)
        };

        let base = Callsign::encode(base_s)?;
        let prefix_raw = if let Some(p) = pfx {
            let mut acc = 0u32;
            for i in 0..4 {
                let ch = if i < p.len() { p[i] } else { b' ' };
                let Some(code) = enc_ch37(ch) else {
                    return Err(CallError::InvalidChar);
                };
                if i < p.len() && code == 0 {
                    return Err(CallError::InvalidChar);
                }
                acc = acc * 37 + code;
            }
            acc
        } else {
            0
        };
        Ok(Self { base, prefix_raw, suffix })
    }

    /// Pack into 53 bits (`bits 52..25` = base, `bits 24..4` = prefix_raw, `bits 3..0` = suffix).
    pub const fn pack_u64(self) -> u64 {
        ((self.base.raw() as u64 & 0x0FFF_FFFF) << 25)
            | ((self.prefix_raw as u64 & 0x001F_FFFF) << 4)
            | (self.suffix as u64 & 0x0F)
    }

    /// Unpack from 53 bits (`None` if base or prefix value is out of range).
    pub const fn unpack_u64(v: u64) -> Option<Self> {
        let b_raw = ((v >> 25) & 0x0FFF_FFFF) as u32;
        let p_raw = ((v >> 4) & 0x001F_FFFF) as u32;
        let suffix = (v & 0x0F) as u8;
        let Some(base) = Callsign::from_raw(b_raw) else {
            return None;
        };
        if p_raw > PREFIX_MAX_RAW {
            return None;
        }
        Some(Self { base, prefix_raw: p_raw, suffix })
    }

    /// Format as `[PREFIX/]BASE[-SUFFIX]` into `out`, returning the byte length.
    pub fn format(self, out: &mut [u8; 16]) -> usize {
        let mut n = 0usize;
        if self.prefix_raw != 0 {
            let mut p = [b' '; 4];
            let mut x = self.prefix_raw;
            for i in (0..4).rev() {
                p[i] = dec_ch37(x % 37);
                x /= 37;
            }
            let plen = p.iter().rposition(|&c| c != b' ').map_or(0, |i| i + 1);
            out[n..n + plen].copy_from_slice(&p[..plen]);
            n += plen;
            out[n] = b'/';
            n += 1;
        }
        let mut base_buf = [0u8; 6];
        let blen = self.base.decode(&mut base_buf);
        out[n..n + blen].copy_from_slice(&base_buf[..blen]);
        n += blen;
        if (1..=15).contains(&self.suffix) {
            out[n] = b'-';
            n += 1;
            if self.suffix >= 10 {
                out[n] = b'1';
                out[n + 1] = b'0' + (self.suffix - 10);
                n += 2;
            } else {
                out[n] = b'0' + self.suffix;
                n += 1;
            }
        }
        n
    }
}

fn trim_ascii(mut s: &[u8]) -> &[u8] {
    while let Some((&b, rest)) = s.split_first() {
        if b == b' ' {
            s = rest;
        } else {
            break;
        }
    }
    while let Some((&b, rest)) = s.split_last() {
        if b == b' ' {
            s = rest;
        } else {
            break;
        }
    }
    s
}

/// CCITT CRC-16 (poly `0x1021`, init `0xFFFF`) over a byte slice.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if (crc & 0x8000) != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// Reassembled decorated-identity logical message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtMsg {
    Cq {
        sender: StationId,
        grid4: u16,
        tx_pwr: u8,
    },
    Rx {
        sender: StationId,
        recipient: StationId,
        snr_db: i8,
        ack: bool,
    },
}

/// Pack `ExtMsg::Cq` into 3 `Frame::ExtFrag` wire frames.
/// Bit layout (135 bits across 3 x 45-bit fragments):
/// - `f0` (45 b): `sender[52..8]`
/// - `f1` (45 b): `sender[7..0]` (8 b) | `grid4` (15 b) | `tx_pwr` (4 b) | `reserved=0` (18 b)
/// - `f2` (45 b): `crc16` (16 b) | `reserved=0` (29 b)
pub fn encode_ext_cq(sender: StationId, grid4: u16, tx_pwr: u8) -> [Frame; CQ_FRAGS] {
    let s53 = sender.pack_u64();
    let f0 = (s53 >> 8) & FRAG_MASK;
    let f1 = ((s53 & 0xFF) << 37) | ((grid4 as u64 & 0x7FFF) << 22) | ((tx_pwr as u64 & 0x0F) << 18);
    let mut crc_in = [0u8; 16];
    crc_in[0..8].copy_from_slice(&f0.to_be_bytes());
    crc_in[8..16].copy_from_slice(&f1.to_be_bytes());
    let crc = crc16_ccitt(&crc_in);
    let f2 = (crc as u64) << 29;
    let tag = ((crc ^ (crc >> 8)) & 0xFF) as u8;
    [
        Frame::ExtFrag { kind: EXT_KIND_CQ, idx: 0, tag, payload: f0 },
        Frame::ExtFrag { kind: EXT_KIND_CQ, idx: 1, tag, payload: f1 },
        Frame::ExtFrag { kind: EXT_KIND_CQ, idx: 2, tag, payload: f2 },
    ]
}

/// Pack `ExtMsg::Rx` into 4 `Frame::ExtFrag` wire frames.
/// Bit layout (180 bits across 4 x 45-bit fragments):
/// - `f0` (45 b): `sender[52..8]`
/// - `f1` (45 b): `sender[7..0]` (8 b) | `recipient[52..16]` (37 b)
/// - `f2` (45 b): `recipient[15..0]` (16 b) | `snr` (5 b) | `ack` (1 b) | `reserved=0` (23 b)
/// - `f3` (45 b): `crc16` (16 b) | `reserved=0` (29 b)
pub fn encode_ext_rx(sender: StationId, recipient: StationId, snr_db: i8, ack: bool) -> [Frame; RX_FRAGS] {
    let s53 = sender.pack_u64();
    let r53 = recipient.pack_u64();
    let snr = encode_snr_db(snr_db) as u64 & 0x1F;
    let f0 = (s53 >> 8) & FRAG_MASK;
    let f1 = ((s53 & 0xFF) << 37) | ((r53 >> 16) & ((1u64 << 37) - 1));
    let f2 = ((r53 & 0xFFFF) << 29) | (snr << 24) | ((ack as u64) << 23);
    let mut crc_in = [0u8; 24];
    crc_in[0..8].copy_from_slice(&f0.to_be_bytes());
    crc_in[8..16].copy_from_slice(&f1.to_be_bytes());
    crc_in[16..24].copy_from_slice(&f2.to_be_bytes());
    let crc = crc16_ccitt(&crc_in);
    let f3 = (crc as u64) << 29;
    let tag = ((crc ^ (crc >> 8)) & 0xFF) as u8;
    let kind = if ack { EXT_KIND_RX_ACK } else { EXT_KIND_RX };
    [
        Frame::ExtFrag { kind, idx: 0, tag, payload: f0 },
        Frame::ExtFrag { kind, idx: 1, tag, payload: f1 },
        Frame::ExtFrag { kind, idx: 2, tag, payload: f2 },
        Frame::ExtFrag { kind, idx: 3, tag, payload: f3 },
    ]
}

fn decode_ext_cq(frags: &[u64; 3]) -> Option<ExtMsg> {
    let mut crc_in = [0u8; 16];
    crc_in[0..8].copy_from_slice(&frags[0].to_be_bytes());
    crc_in[8..16].copy_from_slice(&frags[1].to_be_bytes());
    let want = crc16_ccitt(&crc_in);
    let got = ((frags[2] >> 29) & 0xFFFF) as u16;
    if got != want {
        return None;
    }
    let s53 = (frags[0] << 8) | ((frags[1] >> 37) & 0xFF);
    let sender = StationId::unpack_u64(s53)?;
    let grid4 = ((frags[1] >> 22) & 0x7FFF) as u16;
    if grid4 > GRID_MAX && grid4 != crate::frame::GRID_UNKNOWN {
        return None;
    }
    let tx_pwr = ((frags[1] >> 18) & 0x0F) as u8;
    Some(ExtMsg::Cq { sender, grid4, tx_pwr })
}

fn decode_ext_rx(frags: &[u64; 4], kind: u8) -> Option<ExtMsg> {
    let mut crc_in = [0u8; 24];
    crc_in[0..8].copy_from_slice(&frags[0].to_be_bytes());
    crc_in[8..16].copy_from_slice(&frags[1].to_be_bytes());
    crc_in[16..24].copy_from_slice(&frags[2].to_be_bytes());
    let want = crc16_ccitt(&crc_in);
    let got = ((frags[3] >> 29) & 0xFFFF) as u16;
    if got != want {
        return None;
    }
    let s53 = (frags[0] << 8) | ((frags[1] >> 37) & 0xFF);
    let r53 = ((frags[1] & ((1u64 << 37) - 1)) << 16) | ((frags[2] >> 29) & 0xFFFF);
    let sender = StationId::unpack_u64(s53)?;
    let recipient = StationId::unpack_u64(r53)?;
    let snr_db = decode_snr_db(((frags[2] >> 24) & 0x1F) as u8);
    let ack = ((frags[2] >> 23) & 1) != 0;
    if ack != (kind == EXT_KIND_RX_ACK) {
        return None;
    }
    Some(ExtMsg::Rx { sender, recipient, snr_db, ack })
}

const REASM_SLOTS: usize = 8;
const REASM_TTL_SLOTS: u32 = 12;

#[derive(Clone, Copy)]
struct ReasmEntry {
    active: bool,
    kind: u8,
    tag: u8,
    mask: u8,
    last_slot: u32,
    frags: [u64; 4],
}

impl ReasmEntry {
    const EMPTY: Self = Self {
        active: false,
        kind: 0,
        tag: 0,
        mask: 0,
        last_slot: 0,
        frags: [0; 4],
    };
}

/// Bounded, zero-heap reassembler for multi-fragment decorated station identity messages.
pub struct ExtReassembler {
    entries: [ReasmEntry; REASM_SLOTS],
}

impl Default for ExtReassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtReassembler {
    pub const fn new() -> Self {
        Self { entries: [ReasmEntry::EMPTY; REASM_SLOTS] }
    }

    /// Evict entries older than `REASM_TTL_SLOTS`.
    pub fn expire(&mut self, slot: u32) {
        for e in &mut self.entries {
            if e.active && slot.saturating_sub(e.last_slot) > REASM_TTL_SLOTS {
                *e = ReasmEntry::EMPTY;
            }
        }
    }

    /// Push a decoded `Frame::ExtFrag` at UTC `slot`. Returns `Some(ExtMsg)` once all fragments
    /// have arrived and the 16-bit logical CRC verifies.
    pub fn push(&mut self, slot: u32, kind: u8, idx: u8, tag: u8, payload: u64) -> Option<ExtMsg> {
        self.expire(slot);
        let need_mask: u8 = match kind {
            EXT_KIND_CQ if (idx as usize) < CQ_FRAGS => 0b0111,
            EXT_KIND_RX | EXT_KIND_RX_ACK if (idx as usize) < RX_FRAGS => 0b1111,
            _ => return None,
        };
        let pos = self
            .entries
            .iter()
            .position(|e| e.active && e.kind == kind && e.tag == tag)
            .or_else(|| self.entries.iter().position(|e| !e.active))
            .unwrap_or_else(|| {
                // Evict oldest entry if table is full.
                self.entries
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, e)| e.last_slot)
                    .map_or(0, |(i, _)| i)
            });

        let e = &mut self.entries[pos];
        if !e.active || e.kind != kind || e.tag != tag {
            *e = ReasmEntry {
                active: true,
                kind,
                tag,
                mask: 0,
                last_slot: slot,
                frags: [0; 4],
            };
        }
        e.last_slot = slot;
        e.frags[idx as usize] = payload & FRAG_MASK;
        e.mask |= 1u8 << idx;

        if e.mask == need_mask {
            let frags = e.frags;
            *e = ReasmEntry::EMPTY;
            match kind {
                EXT_KIND_CQ => decode_ext_cq(&[frags[0], frags[1], frags[2]]),
                EXT_KIND_RX | EXT_KIND_RX_ACK => decode_ext_rx(&frags, kind),
                _ => None,
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::encode_grid4;

    #[test]
    fn station_id_round_trip_and_distinctness() {
        let cases: &[&[u8]] = &[b"NQ6N", b"NQ6N-1", b"NQ6N-2", b"NQ6N-15", b"KP2/NQ6N", b"KP2/NQ6N-1", b"VE3/W1AW-12"];
        for &s in cases {
            let id = StationId::parse(s).expect("valid station id");
            let packed = id.pack_u64();
            assert!(packed < (1u64 << 53), "must fit in 53 bits");
            let unpacked = StationId::unpack_u64(packed).unwrap();
            assert_eq!(id, unpacked);
            let mut buf = [0u8; 16];
            let n = unpacked.format(&mut buf);
            assert_eq!(&buf[..n], s);
        }
        let s1 = StationId::parse(b"NQ6N-1").unwrap();
        let s2 = StationId::parse(b"NQ6N-2").unwrap();
        assert_ne!(s1, s2, "NQ6N-1 and NQ6N-2 must remain distinct identities");
        assert!(s1.is_decorated());
        assert!(!StationId::parse(b"NQ6N").unwrap().is_decorated());
        assert!(StationId::parse(b"NQ6N-0").is_err(), "suffix 0 invalid");
        assert!(StationId::parse(b"NQ6N-16").is_err(), "suffix > 15 invalid");
    }

    #[test]
    fn ext_cq_and_rx_wire_round_trip_and_legacy_rejection() {
        let me = StationId::parse(b"KP2/NQ6N-1").unwrap();
        let peer = StationId::parse(b"DL/K1ABC-2").unwrap();
        let fk77 = encode_grid4(b"FK77").unwrap();

        // 3-fragment CQ
        let cq_frags = encode_ext_cq(me, fk77, 10);
        let mut reasm = ExtReassembler::new();
        // Verify each fragment has bit 63=1 and bits 62..57 = 0b111111 (strictly rejected by legacy Callsign::from_raw).
        for (i, &f) in cq_frags.iter().enumerate() {
            let w = f.pack_u64();
            assert_eq!((w >> 63) & 1, 1);
            assert_eq!((w >> 57) & 0x3F, EXT_MARKER_BITS);
            let legacy_sender_raw = ((w >> 35) & 0x0FFF_FFFF) as u32;
            assert!(Callsign::from_raw(legacy_sender_raw).is_none(), "legacy decoder must reject ext frag");
            let bits = f.to_bits78();
            let decoded = Frame::from_bits78(&bits).expect("valid CRC-14 + ext frame");
            assert_eq!(decoded, f);
            let Frame::ExtFrag { kind, idx, tag, payload } = decoded else { panic!("expected ExtFrag") };
            let got = reasm.push(100 + i as u32, kind, idx, tag, payload);
            if i < 2 {
                assert!(got.is_none());
            } else {
                assert_eq!(got, Some(ExtMsg::Cq { sender: me, grid4: fk77, tx_pwr: 10 }));
            }
        }

        // 4-fragment directed RX (out-of-order arrival: 2, 0, 3, 1)
        let rx_frags = encode_ext_rx(me, peer, -14, true);
        let order = [2usize, 0, 3, 1];
        for (step, &k) in order.iter().enumerate() {
            let Frame::ExtFrag { kind, idx, tag, payload } = Frame::unpack_u64(rx_frags[k].pack_u64()).unwrap() else {
                panic!("expected ExtFrag");
            };
            let got = reasm.push(200 + step as u32, kind, idx, tag, payload);
            if step < 3 {
                assert!(got.is_none());
            } else {
                assert_eq!(got, Some(ExtMsg::Rx { sender: me, recipient: peer, snr_db: -14, ack: true }));
            }
        }

        // Erased fragment: only 3 of 4 RX fragments arrive -> never emits, expires cleanly.
        for &k in &[0usize, 1, 3] {
            let Frame::ExtFrag { kind, idx, tag, payload } = rx_frags[k] else { unreachable!() };
            assert!(reasm.push(300, kind, idx, tag, payload).is_none());
        }
        // After expiry, pushing frag 2 alone must not reassemble stale fragments.
        let Frame::ExtFrag { kind, idx, tag, payload } = rx_frags[2] else { unreachable!() };
        assert!(reasm.push(300 + REASM_TTL_SLOTS + 2, kind, idx, tag, payload).is_none());
    }
}
