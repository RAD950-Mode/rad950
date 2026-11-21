//! RAD950 64-bit `CQ` and `RX` frame packing, standard 28-bit amateur callsign codec, Maidenhead `GRID4`,
//! quantized `SNR` / `TX_PWR`, and CRC-14 (`0x2757`).

/// Sentinel in the 15-bit `GRID4` field meaning "grid unknown / no GPS fix".
pub const GRID_UNKNOWN: u16 = 32_400;
/// Maximum valid Maidenhead 4-character grid value (`RR99` = 32,399).
pub const GRID_MAX: u16 = 32_399;

/// CRC-14 generator polynomial (`x^14 + x^13 + x^10 + x^8 + x^6 + x^4 + x^2 + x + 1`, `0x2757`, FT8/FT4 family).
pub const CRC14_POLY: u16 = 0x2757;

/// Standard amateur callsign packed into 28 bits (`0 .. 262_177_560`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Callsign(u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallError {
    Empty,
    TooLong,
    InvalidChar,
    NoAreaDigit,
    UnsupportedFormat,
}

impl Callsign {
    pub const MAX_RAW: u32 = 37 * 36 * 10 * 27 * 27 * 27 - 1;

    pub const fn from_raw(raw: u32) -> Option<Self> {
        if raw <= Self::MAX_RAW {
            Some(Self(raw))
        } else {
            None
        }
    }

    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Encode a standard amateur callsign (e.g. `b"W1AW"`, `b"K1ABC"`, `b"VE3XYZ"`, `b"3DA0RS"`).
    ///
    /// Alignment follows the standard 6-slot `[C0, C1, C2, C3, C4, C5]` rule where `C2` is the call-area digit:
    /// - `C0`: `' '`, `'A'..='Z'`, `'0'..='9'` (37)
    /// - `C1`: `'A'..='Z'`, `'0'..='9'` (36)
    /// - `C2`: `'0'..='9'` (10)
    /// - `C3..=C5`: `' '`, `'A'..='Z'` (27 each, trailing spaces)
    pub fn encode(s: &[u8]) -> Result<Self, CallError> {
        let mut trimmed = s;
        while let Some((&b' ', rest)) = trimmed.split_first() {
            trimmed = rest;
        }
        while let Some((&b' ', rest)) = trimmed.split_last() {
            trimmed = rest;
        }
        if trimmed.is_empty() {
            return Err(CallError::Empty);
        }
        if trimmed.len() > 6 {
            return Err(CallError::TooLong);
        }

        let mut up = [b' '; 6];
        for (i, &b) in trimmed.iter().enumerate() {
            let c = b.to_ascii_uppercase();
            if !c.is_ascii_alphanumeric() {
                return Err(CallError::InvalidChar);
            }
            up[i] = c;
        }
        let len = trimmed.len();

        // Find the area digit: the last digit in positions 1..min(len, 3) such that remaining suffix is all letters
        // and <= 3 chars. Handles 1x2/1x3 (W1AW -> ' W1AW '), 2x1/2x2/2x3 (K1ABC/AB1CD), and 3DA0RS ('3DA0RS' — note
        // 3DA0 has digit at index 3? No, standard 28-bit slots put C0='3', C1='D' only if digit is at C2, so max 2-char
        // prefix before the area digit: e.g. G4ABC, VE3XYZ, 9A1A, 3D2AG).
        let mut slots = [b' '; 6];
        let mut placed = false;
        for d_pos in (1..len.min(3)).rev() {
            if up[d_pos].is_ascii_digit() && (len - 1 - d_pos) <= 3 && up[d_pos + 1..len].iter().all(|c| c.is_ascii_uppercase()) {
                let shift = 2 - d_pos;
                slots[shift..(len + shift)].copy_from_slice(&up[..len]);
                placed = true;
                break;
            }
        }
        if !placed {
            if up.iter().take(len).any(|c| c.is_ascii_digit()) {
                return Err(CallError::UnsupportedFormat);
            }
            return Err(CallError::NoAreaDigit);
        }

        let c0 = val_c0(slots[0]).ok_or(CallError::UnsupportedFormat)?;
        let c1 = val_c1(slots[1]).ok_or(CallError::UnsupportedFormat)?;
        let c2 = val_c2(slots[2]).ok_or(CallError::UnsupportedFormat)?;
        let c3 = val_suffix(slots[3]).ok_or(CallError::UnsupportedFormat)?;
        let c4 = val_suffix(slots[4]).ok_or(CallError::UnsupportedFormat)?;
        let c5 = val_suffix(slots[5]).ok_or(CallError::UnsupportedFormat)?;

        let raw = ((((c0 * 36 + c1) * 10 + c2) * 27 + c3) * 27 + c4) * 27 + c5;
        Ok(Self(raw))
    }

    /// Decode into `out` (up to 6 ASCII bytes), returning the trimmed length.
    pub fn decode(self, out: &mut [u8; 6]) -> usize {
        let mut v = self.0;
        let c5 = (v % 27) as u8;
        v /= 27;
        let c4 = (v % 27) as u8;
        v /= 27;
        let c3 = (v % 27) as u8;
        v /= 27;
        let c2 = (v % 10) as u8;
        v /= 10;
        let c1 = (v % 36) as u8;
        v /= 36;
        let c0 = v as u8;

        let raw_chars = [
            ch_c0(c0),
            ch_c1(c1),
            b'0' + c2,
            ch_suffix(c3),
            ch_suffix(c4),
            ch_suffix(c5),
        ];
        let mut n = 0;
        for &c in &raw_chars {
            if c != b' ' {
                out[n] = c;
                n += 1;
            }
        }
        for b in &mut out[n..] {
            *b = b' ';
        }
        n
    }
}

const fn val_c0(c: u8) -> Option<u32> {
    match c {
        b' ' => Some(0),
        b'A'..=b'Z' => Some(1 + (c - b'A') as u32),
        b'0'..=b'9' => Some(27 + (c - b'0') as u32),
        _ => None,
    }
}

const fn ch_c0(v: u8) -> u8 {
    match v {
        0 => b' ',
        1..=26 => b'A' + (v - 1),
        _ => b'0' + (v - 27),
    }
}

const fn val_c1(c: u8) -> Option<u32> {
    match c {
        b'A'..=b'Z' => Some((c - b'A') as u32),
        b'0'..=b'9' => Some(26 + (c - b'0') as u32),
        _ => None,
    }
}

const fn ch_c1(v: u8) -> u8 {
    match v {
        0..=25 => b'A' + v,
        _ => b'0' + (v - 26),
    }
}

const fn val_c2(c: u8) -> Option<u32> {
    match c {
        b'0'..=b'9' => Some((c - b'0') as u32),
        _ => None,
    }
}

const fn val_suffix(c: u8) -> Option<u32> {
    match c {
        b' ' => Some(0),
        b'A'..=b'Z' => Some(1 + (c - b'A') as u32),
        _ => None,
    }
}

const fn ch_suffix(v: u8) -> u8 {
    match v {
        0 => b' ',
        _ => b'A' + (v - 1),
    }
}

/// Encode a 4-character Maidenhead grid (`b"FN42"` -> `0..=32399`).
pub fn encode_grid4(g: &[u8; 4]) -> Option<u16> {
    let f0 = g[0].to_ascii_uppercase();
    let f1 = g[1].to_ascii_uppercase();
    if !(b'A'..=b'R').contains(&f0) || !(b'A'..=b'R').contains(&f1) {
        return None;
    }
    if !g[2].is_ascii_digit() || !g[3].is_ascii_digit() {
        return None;
    }
    let v = (((f0 - b'A') as u16 * 18 + (f1 - b'A') as u16) * 10 + (g[2] - b'0') as u16) * 10 + (g[3] - b'0') as u16;
    Some(v)
}

/// Decode a 15-bit `GRID4` field (`None` for [`GRID_UNKNOWN`] or reserved values).
pub fn decode_grid4(v: u16) -> Option<[u8; 4]> {
    if v > GRID_MAX {
        return None;
    }
    let d1 = (v % 10) as u8;
    let d0 = ((v / 10) % 10) as u8;
    let f1 = ((v / 100) % 18) as u8;
    let f0 = (v / 1800) as u8;
    Some([b'A' + f0, b'A' + f1, b'0' + d0, b'0' + d1])
}

/// Quantize SNR (dB in 2,500 Hz reference bandwidth) to 5 bits (`[-24, +7]` dB in 1 dB steps).
pub const fn encode_snr_db(snr_db: i8) -> u8 {
    let clamped = if snr_db < -24 {
        -24
    } else if snr_db > 7 {
        7
    } else {
        snr_db
    };
    (clamped + 24) as u8
}

/// Decode 5-bit SNR code (`0..=31`) back to dB (`-24..=+7` dB).
pub const fn decode_snr_db(code: u8) -> i8 {
    (code & 0x1F) as i8 - 24
}

/// RAD950 over-the-air 64-bit payload frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame {
    /// Type 0: `CQ` / beacon (`1b type=0 | 28b sender | 15b grid4 | 4b tx_pwr | 16b reserved`).
    Cq {
        sender: Callsign,
        grid4: u16,
        tx_pwr: u8,
        reserved: u16,
    },
    /// Type 1: Directed `RX` / `R+RX` report (`1b type=1 | 28b sender | 28b recipient | 5b snr | 1b ack | 1b reserved`).
    Rx {
        sender: Callsign,
        recipient: Callsign,
        snr_db: i8,
        ack: bool,
        reserved: bool,
    },
    /// Extension fragment (`bit 63 = 1 | bits 62..57 = 0b111111 | 2b kind | 2b idx | 8b tag | 45b payload`).
    /// Strictly rejected by legacy decoders because `s_raw >= 0x0FC0_0000 > Callsign::MAX_RAW`.
    ExtFrag {
        kind: u8,
        idx: u8,
        tag: u8,
        payload: u64,
    },
}

impl Frame {
    /// Pack into a 64-bit unsigned integer (MSB = `TYPE`).
    pub const fn pack_u64(self) -> u64 {
        match self {
            Frame::Cq { sender, grid4, tx_pwr, reserved } => {
                ((sender.raw() as u64 & 0x0FFF_FFFF) << 35)
                    | ((grid4 as u64 & 0x7FFF) << 20)
                    | ((tx_pwr as u64 & 0x0F) << 16)
                    | (reserved as u64)
            }
            Frame::Rx { sender, recipient, snr_db, ack, reserved } => {
                let snr_code = encode_snr_db(snr_db) as u64 & 0x1F;
                (1u64 << 63)
                    | ((sender.raw() as u64 & 0x0FFF_FFFF) << 35)
                    | ((recipient.raw() as u64 & 0x0FFF_FFFF) << 7)
                    | (snr_code << 2)
                    | ((ack as u64) << 1)
                    | (reserved as u64)
            }
            Frame::ExtFrag { kind, idx, tag, payload } => {
                (1u64 << 63)
                    | (crate::ext::EXT_MARKER_BITS << 57)
                    | ((kind as u64 & 0x03) << 55)
                    | ((idx as u64 & 0x03) << 53)
                    | ((tag as u64 & 0xFF) << 45)
                    | (payload & crate::ext::FRAG_MASK)
            }
        }
    }

    /// Unpack a 64-bit payload (`None` if either 28-bit callsign value exceeds [`Callsign::MAX_RAW`]).
    pub const fn unpack_u64(w: u64) -> Option<Self> {
        let typ = (w >> 63) & 1;
        if typ == 1 && ((w >> 57) & 0x3F) == crate::ext::EXT_MARKER_BITS {
            let kind = ((w >> 55) & 0x03) as u8;
            let idx = ((w >> 53) & 0x03) as u8;
            let tag = ((w >> 45) & 0xFF) as u8;
            let payload = w & crate::ext::FRAG_MASK;
            return Some(Frame::ExtFrag { kind, idx, tag, payload });
        }
        let s_raw = ((w >> 35) & 0x0FFF_FFFF) as u32;
        let Some(sender) = Callsign::from_raw(s_raw) else {
            return None;
        };
        if typ == 0 {
            let grid4 = ((w >> 20) & 0x7FFF) as u16;
            let tx_pwr = ((w >> 16) & 0x0F) as u8;
            let reserved = (w & 0xFFFF) as u16;
            Some(Frame::Cq { sender, grid4, tx_pwr, reserved })
        } else {
            let r_raw = ((w >> 7) & 0x0FFF_FFFF) as u32;
            let Some(recipient) = Callsign::from_raw(r_raw) else {
                return None;
            };
            let snr_db = decode_snr_db(((w >> 2) & 0x1F) as u8);
            let ack = ((w >> 1) & 1) != 0;
            let reserved = (w & 1) != 0;
            Some(Frame::Rx { sender, recipient, snr_db, ack, reserved })
        }
    }

    /// Pack into 78 systematic bits (`[0..64]` MSB-first payload + `[64..78]` MSB-first CRC-14).
    pub fn to_bits78(self) -> [u8; 78] {
        let w = self.pack_u64();
        let crc = crc14_u64(w);
        let mut bits = [0u8; 78];
        for i in 0..64 {
            bits[i] = ((w >> (63 - i)) & 1) as u8;
        }
        for i in 0..14 {
            bits[64 + i] = ((crc >> (13 - i)) & 1) as u8;
        }
        bits
    }

    /// Verify CRC-14 and unpack 78 systematic bits into a [`Frame`].
    pub fn from_bits78(bits: &[u8; 78]) -> Option<Self> {
        let mut w = 0u64;
        for i in 0..64 {
            w = (w << 1) | (bits[i] as u64 & 1);
        }
        let mut rx_crc = 0u16;
        for i in 0..14 {
            rx_crc = (rx_crc << 1) | (bits[64 + i] as u16 & 1);
        }
        // Reject all-zero payload+CRC so unmodulated silence never decodes as a valid frame (`00..0` has CRC 0
        // unless CRC is non-zero initialized or XORed; we XOR final CRC with 0x1555 in crc14_u64 so all-zeros fails!).
        if crc14_u64(w) != rx_crc {
            return None;
        }
        Self::unpack_u64(w)
    }
}

/// CRC-14 over a 64-bit MSB-first word using polynomial `0x2757`, init `0x3FFF` (prevents all-zero false codewords).
pub const fn crc14_u64(w: u64) -> u16 {
    let mut reg: u16 = 0x3FFF;
    let mut i = 0;
    while i < 64 {
        let bit = ((w >> (63 - i)) as u16) & 1;
        let top = (reg >> 13) & 1;
        reg = (reg << 1) & 0x3FFF;
        if (bit ^ top) != 0 {
            reg ^= CRC14_POLY;
        }
        i += 1;
    }
    reg
}
