//! RAD950 Conversation Extension (`docs/rad950-conversation-extension.md`).
//!
//! Uses `Frame::ExtFrag { kind: CHAT_EXT_KIND (2), .. }` (`bit 63 = 1, bits 62..57 = 0b111111`), which is
//! strictly rejected by legacy v2 decoders and shares the extension envelope cleanly with `ext.rs` (`kind = 0, 1`).
//!
//! 55-bit layout inside `(idx: 2b, tag: 8b, payload: 45b)`:
//! - `bits 54..52` (3 bits): `msg_type` (`0` Control, `1` StationCard, `2` Question, `3` Phrase, `4` LiteralFrag)
//! - `bits 51..41` (11 bits): `session_ref` (session binding reference)
//! - `bits 40..38` (3 bits): `seq` (per-direction message sequence `0..7`)
//! - `bits 37..0`  (38 bits): typed card / phrase / literal fragment body

use crate::frame::Frame;

/// Extension `kind` reserved for RAD950 Conversation frames inside `Frame::ExtFrag`.
pub const CHAT_EXT_KIND: u8 = 2;

/// 6-bit literal text alphabet (64 symbols: `0` = pad/space, `1..26` = `A..Z`, `27..36` = `0..9`, punctuation).
const ALPHABET: &[u8; 64] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,?!/-:+()@'\"#&*=_%<>[]\\;^~";

/// Maximum literal message length (`8` fragments x `5` chars = `40` characters).
pub const CHAT_LITERAL_MAX: usize = 40;
/// Maximum fragments per literal message.
pub const CHAT_MAX_FRAGS: usize = 8;

/// Saved operator station card (`38` bits on the air — fits in a single 10 s burst).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StationCard {
    /// Catalog rig ID (`0` = Unspecified, `1` = RT-950 Pro, `2` = Portable QRP, `3` = Base HF).
    pub rig: u16,
    /// Indicated RF output power in dBm (`0..=60`, `127` = unknown; `37 dBm` ≈ `5 W`).
    pub pwr_dbm: u8,
    /// External amplifier on (`true`) or barefoot (`false`).
    pub amp: bool,
    /// Catalog antenna ID (`0` = Unspecified, `1` = Dipole, `2` = Whip, `3` = EFHW, `4` = Vertical, `5` = Yagi).
    pub ant: u8,
    /// Antenna height above local ground in whole meters (`0..=126`, `127` = unknown).
    pub height_m: u8,
    /// Local temperature in whole degrees Celsius (`-60..=+65`, `i8::MIN` = unknown).
    pub temp_c: i8,
}

impl StationCard {
    pub const DEFAULT_RT950: Self = Self {
        rig: 1,       // RT-950 Pro
        pwr_dbm: 37,  // 5 W
        amp: false,   // barefoot
        ant: 1,       // Dipole
        height_m: 8,  // 8 m
        temp_c: 18,   // 18 C
    };

    pub fn pack38(&self) -> u64 {
        let rig = (self.rig as u64) & 0x3FF;
        let pwr = (self.pwr_dbm as u64) & 0x7F;
        let amp = (self.amp as u64) & 1;
        let ant = (self.ant as u64) & 0x1F;
        let ht = (self.height_m as u64) & 0x7F;
        let tc = if self.temp_c == i8::MIN {
            0xFFu64
        } else {
            ((self.temp_c as i32 + 60).clamp(0, 254) as u64) & 0xFF
        };
        (rig << 28) | (pwr << 21) | (amp << 20) | (ant << 15) | (ht << 8) | tc
    }

    pub fn unpack38(v: u64) -> Self {
        let rig = ((v >> 28) & 0x3FF) as u16;
        let pwr_dbm = ((v >> 21) & 0x7F) as u8;
        let amp = ((v >> 20) & 1) != 0;
        let ant = ((v >> 15) & 0x1F) as u8;
        let height_m = ((v >> 8) & 0x7F) as u8;
        let tc_raw = (v & 0xFF) as u8;
        let temp_c = if tc_raw == 0xFF { i8::MIN } else { (tc_raw as i32 - 60).clamp(-60, 127) as i8 };
        Self { rig, pwr_dbm, amp, ant, height_m, temp_c }
    }
}

/// Local weather card (`38` bits on the air — fits in a single 10 s burst).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WeatherCard {
    /// Condition ID (`0` = Unspecified, `1` = Clear, `2` = Cloudy, `3` = Light rain, `4` = Rain, `5` = Snow, `6` = Fog, `7` = Storm).
    pub cond: u8,
    /// Temperature in whole degrees Celsius (`-60..=+65`, `i8::MIN` = unknown).
    pub temp_c: i8,
    /// Wind speed in km/h (`0..=126`, `127` = unknown).
    pub wind_kph: u8,
    /// Wind direction (`0` = unknown, `1..=8` = N, NE, E, SE, S, SW, W, NW).
    pub wind_dir: u8,
}

impl WeatherCard {
    pub const DEFAULT: Self = Self {
        cond: 3,      // Light rain
        temp_c: 12,   // 12 C
        wind_kph: 15, // 15 km/h
        wind_dir: 8,  // NW
    };

    pub fn pack38(&self) -> u64 {
        let cond = (self.cond as u64) & 0x1F;
        let tc = if self.temp_c == i8::MIN {
            0xFFu64
        } else {
            ((self.temp_c as i32 + 60).clamp(0, 254) as u64) & 0xFF
        };
        let wk = (self.wind_kph as u64) & 0x7F;
        let wd = (self.wind_dir as u64) & 0x0F;
        (cond << 19) | (tc << 11) | (wk << 4) | wd
    }

    pub fn unpack38(v: u64) -> Self {
        let cond = ((v >> 19) & 0x1F) as u8;
        let tc_raw = ((v >> 11) & 0xFF) as u8;
        let temp_c = if tc_raw == 0xFF { i8::MIN } else { (tc_raw as i32 - 60).clamp(-60, 127) as i8 };
        let wind_kph = ((v >> 4) & 0x7F) as u8;
        let wind_dir = (v & 0x0F) as u8;
        Self { cond, temp_c, wind_kph, wind_dir }
    }
}

/// Optional operator "About me" card (`38` bits on the air — fits in a single 10 s burst).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AboutCard {
    /// Optional operator age (`0` = omitted, `1..=126`).
    pub age: u8,
    /// Optional year first licensed (`0` = omitted, `1920..=2174`).
    pub year_licensed: u16,
}

impl AboutCard {
    pub const DEFAULT: Self = Self { age: 0, year_licensed: 1994 };

    pub fn pack38(&self) -> u64 {
        let age = (self.age.min(126) as u64) & 0x7F;
        let yr = if self.year_licensed >= 1920 && self.year_licensed <= 2174 {
            ((self.year_licensed - 1920 + 1) as u64) & 0xFF
        } else {
            0
        };
        (age << 8) | yr
    }

    pub fn unpack38(v: u64) -> Self {
        let age = ((v >> 8) & 0x7F) as u8;
        let yr_raw = (v & 0xFF) as u16;
        let year_licensed = if yr_raw == 0 { 0 } else { 1920 + yr_raw - 1 };
        Self { age, year_licensed }
    }
}

/// Control frame subtypes (`msg_type = 0`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChatCtrl {
    /// Invite peer to chat on `calling_dial + qsy_step * 4 kHz` (`0` = stay, `1` = +4 kHz default) and `subch` (`0..=7`).
    Invite { qsy_step: u8, subch: u8 },
    Accept,
    Decline,
    End,
    Ack { ack_seq: u8, frag_mask: u8 },
}

/// Compute the chat USB dial in 10 Hz units (`calling_dial + qsy_step * 4 kHz`).
pub const fn chat_qsy_dial_hz10(calling_dial_hz10: u32, qsy_step: u8) -> u32 {
    calling_dial_hz10.saturating_add((qsy_step as u32 & 7) * 400)
}

/// Compute the audio subchannel center frequency (Hz) for `subch` (`0..=7`) on the 225 Hz grid.
pub const fn chat_subch_hz(subch: u8) -> u16 {
    625 + ((subch as u16) & 7) * 225
}

/// Action on a directed Rendezvous offer (`Schedule QSO` or `Move to frequency`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RendezvousAction {
    Offer = 0,
    Accept = 1,
    Counter = 2,
    Decline = 3,
}

impl RendezvousAction {
    pub const fn from_u8(v: u8) -> Self {
        match v & 3 {
            0 => Self::Offer,
            1 => Self::Accept,
            2 => Self::Counter,
            _ => Self::Decline,
        }
    }
}

/// Directed rendezvous offer (`38` bits on the air — fits in a single 10 s burst).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RendezvousCard {
    /// Schedule a future QSO (`subtype = 0`):
    /// `1b subtype | 2b action | 7b days_ahead | 11b utc_min | 8b dial_preset | 3b profile | 3b subch | 3b offer_ref`.
    Schedule {
        action: RendezvousAction,
        days_ahead: u8,
        utc_min: u16,
        dial_preset: u8,
        profile: u8,
        subch: u8,
        offer_ref: u8,
    },
    /// Near-term move to another frequency (`subtype = 1`):
    /// `1b subtype | 2b action | 5b band_id | 18b dial_offset_10hz | 3b profile | 6b turn_mod64 | 3b subch`.
    MoveFreq {
        action: RendezvousAction,
        band_id: u8,
        dial_offset_10hz: u32,
        profile: u8,
        turn_mod64: u8,
        subch: u8,
    },
}

impl RendezvousCard {
    pub fn pack38(&self) -> u64 {
        match *self {
            Self::Schedule { action, days_ahead, utc_min, dial_preset, profile, subch, offer_ref } => {
                (((action as u64) & 3) << 35)
                    | (((days_ahead as u64) & 0x7F) << 28)
                    | (((utc_min.min(1439) as u64) & 0x7FF) << 17)
                    | (((dial_preset as u64) & 0xFF) << 9)
                    | (((profile as u64) & 7) << 6)
                    | (((subch as u64) & 7) << 3)
                    | ((offer_ref as u64) & 7)
            }
            Self::MoveFreq { action, band_id, dial_offset_10hz, profile, turn_mod64, subch } => {
                (1u64 << 37)
                    | (((action as u64) & 3) << 35)
                    | (((band_id as u64) & 0x1F) << 30)
                    | (((dial_offset_10hz as u64) & 0x3_FFFF) << 12)
                    | (((profile as u64) & 7) << 9)
                    | (((turn_mod64 as u64) & 0x3F) << 3)
                    | ((subch as u64) & 7)
            }
        }
    }

    pub fn unpack38(v: u64) -> Option<Self> {
        let subtype = ((v >> 37) & 1) as u8;
        let action = RendezvousAction::from_u8(((v >> 35) & 3) as u8);
        if subtype == 0 {
            let days_ahead = ((v >> 28) & 0x7F) as u8;
            let utc_min = ((v >> 17) & 0x7FF) as u16;
            if utc_min > 1439 {
                return None;
            }
            let dial_preset = ((v >> 9) & 0xFF) as u8;
            let profile = ((v >> 6) & 7) as u8;
            let subch = ((v >> 3) & 7) as u8;
            let offer_ref = (v & 7) as u8;
            Some(Self::Schedule { action, days_ahead, utc_min, dial_preset, profile, subch, offer_ref })
        } else {
            let band_id = ((v >> 30) & 0x1F) as u8;
            let dial_offset_10hz = ((v >> 12) & 0x3_FFFF) as u32;
            let profile = ((v >> 9) & 7) as u8;
            let turn_mod64 = ((v >> 3) & 0x3F) as u8;
            let subch = (v & 7) as u8;
            Some(Self::MoveFreq { action, band_id, dial_offset_10hz, profile, turn_mod64, subch })
        }
    }
}

/// Band reference USB dial in 10 Hz units for `RendezvousCard::MoveFreq::band_id`.
pub const fn move_band_ref_hz10(band_id: u8) -> Option<u32> {
    match band_id {
        0 => Some(2_800_000), // 10 m (28.000 MHz)
        1 => Some(2_489_000), // 12 m (24.890 MHz)
        2 => Some(2_100_000), // 15 m (21.000 MHz)
        3 => Some(1_806_800), // 17 m (18.068 MHz)
        _ => None,
    }
}

/// Convert a USB dial in 10 Hz units to `(band_id, offset_10hz)` for `MoveFreq`.
pub const fn dial_to_move_band(dial_hz10: u32) -> Option<(u8, u32)> {
    if dial_hz10 >= 2_800_000 && dial_hz10 < 2_970_000 {
        Some((0, dial_hz10 - 2_800_000))
    } else if dial_hz10 >= 2_489_000 && dial_hz10 < 2_499_000 {
        Some((1, dial_hz10 - 2_489_000))
    } else if dial_hz10 >= 2_100_000 && dial_hz10 < 2_145_000 {
        Some((2, dial_hz10 - 2_100_000))
    } else if dial_hz10 >= 1_806_800 && dial_hz10 < 1_816_800 {
        Some((3, dial_hz10 - 1_806_800))
    } else {
        None
    }
}

/// Decoded single-burst conversation payload (`38` bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChatBody {
    Ctrl(ChatCtrl),
    Station(StationCard),
    Question { topic: u8, variant: u8 },
    Phrase(u8),
    LiteralFrag {
        frag_idx: u8,
        is_final: bool,
        char_cnt: u8,
        is_name: bool,
        chars: [u8; 5],
    },
    Weather(WeatherCard),
    About(AboutCard),
    Rendezvous(RendezvousCard),
}

/// Complete 64-bit conversation wire frame (`session_ref`, `seq`, `body`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChatPacket {
    pub session_ref: u16,
    pub seq: u8,
    pub body: ChatBody,
}

/// Catalog of one-burst questions (`topic`, rendered text).
pub const QUESTIONS: &[&str] = &[
    "What antenna are you using?",
    "What rig and power?",
    "How is the weather there?",
    "What is your name?",
    "Where are you operating from?",
];

/// Catalog of one-burst quick phrases (`phrase_id`, rendered text).
pub const PHRASES: &[&str] = &[
    "How have you been?",
    "First QSO on RAD950. Thanks!",
    "Solid copy here, 73!",
    "Great signal on HF!",
    "Good luck with the setup!",
    "Catch you next time, 73!",
];

/// Rig catalog names (`StationCard::rig`).
pub fn rig_name(id: u16) -> &'static str {
    match id {
        1 => "RT-950 Pro",
        2 => "Portable QRP",
        3 => "Base HF",
        _ => "Rig?",
    }
}

/// Antenna catalog names (`StationCard::ant`).
pub fn ant_name(id: u8) -> &'static str {
    match id {
        1 => "dipole",
        2 => "whip",
        3 => "EFHW",
        4 => "vertical",
        5 => "Yagi",
        _ => "ant?",
    }
}

/// Weather condition catalog names (`WeatherCard::cond`).
pub fn wx_cond_name(id: u8) -> &'static str {
    match id {
        1 => "Clear",
        2 => "Cloudy",
        3 => "Light rain",
        4 => "Rain",
        5 => "Snow",
        6 => "Fog",
        7 => "Storm",
        _ => "Weather",
    }
}

/// Wind direction catalog names (`WeatherCard::wind_dir`).
pub fn wind_dir_name(id: u8) -> &'static str {
    match id {
        1 => "N",
        2 => "NE",
        3 => "E",
        4 => "SE",
        5 => "S",
        6 => "SW",
        7 => "W",
        8 => "NW",
        _ => "",
    }
}

/// Compute a 11-bit session reference from two callsigns and the QSO completion slot.
pub fn session_ref_for(a: u32, b: u32, slot: u32) -> u16 {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    let mut h = lo.wrapping_mul(0x9E37_79B1) ^ hi.wrapping_mul(0x85EB_CA77) ^ slot.wrapping_mul(0xC2B2_AE3D);
    h ^= h >> 16;
    let r = (h & 0x7FF) as u16;
    if r == 0 { 1 } else { r }
}

/// Encode a `ChatPacket` into `Frame::ExtFrag { kind: CHAT_EXT_KIND, .. }`.
pub fn encode_chat_frame(pkt: ChatPacket) -> Frame {
    let (mtype, body38): (u64, u64) = match pkt.body {
        ChatBody::Ctrl(c) => {
            let b = match c {
                ChatCtrl::Invite { qsy_step, subch } => {
                    (((qsy_step as u64) & 7) << 4) | (((subch as u64) & 7) << 7)
                }
                ChatCtrl::Accept => 1,
                ChatCtrl::Decline => 2,
                ChatCtrl::End => 3,
                ChatCtrl::Ack { ack_seq, frag_mask } => {
                    4 | (((ack_seq as u64) & 7) << 4) | (((frag_mask as u64) & 0xFF) << 7)
                }
            };
            (0, b)
        }
        ChatBody::Station(sc) => (1, sc.pack38()),
        ChatBody::Question { topic, variant } => (2, ((topic as u64 & 0x1F) << 3) | (variant as u64 & 7)),
        ChatBody::Phrase(id) => (3, id as u64 & 0xFF),
        ChatBody::LiteralFrag { frag_idx, is_final, char_cnt, is_name, chars } => {
            let mut packed = ((frag_idx as u64 & 7) << 35)
                | ((is_final as u64 & 1) << 34)
                | ((char_cnt.min(5) as u64 & 7) << 31)
                | ((is_name as u64 & 1) << 30);
            for (i, &ch) in chars.iter().enumerate() {
                let code = encode_char6(ch) as u64;
                packed |= code << (24 - i * 6);
            }
            (4, packed)
        }
        ChatBody::Weather(wc) => (5, wc.pack38()),
        ChatBody::About(ac) => (6, ac.pack38()),
        ChatBody::Rendezvous(rv) => (7, rv.pack38()),
    };
    let bits55 = ((mtype & 7) << 52)
        | (((pkt.session_ref as u64) & 0x7FF) << 41)
        | (((pkt.seq as u64) & 7) << 38)
        | (body38 & ((1u64 << 38) - 1));
    let idx = ((bits55 >> 53) & 3) as u8;
    let tag = ((bits55 >> 45) & 0xFF) as u8;
    let payload = bits55 & ((1u64 << 45) - 1);
    Frame::ExtFrag { kind: CHAT_EXT_KIND, idx, tag, payload }
}

/// Decode a `ChatPacket` from a `Frame::ExtFrag { kind: CHAT_EXT_KIND, .. }`.
pub fn decode_chat_frame(frame: Frame) -> Option<ChatPacket> {
    let Frame::ExtFrag { kind: CHAT_EXT_KIND, idx, tag, payload } = frame else {
        return None;
    };
    let bits55 = (((idx as u64) & 3) << 53) | (((tag as u64) & 0xFF) << 45) | (payload & ((1u64 << 45) - 1));
    let mtype = ((bits55 >> 52) & 7) as u8;
    let session_ref = ((bits55 >> 41) & 0x7FF) as u16;
    let seq = ((bits55 >> 38) & 7) as u8;
    let body38 = bits55 & ((1u64 << 38) - 1);
    let body = match mtype {
        0 => {
            let ctrl = match body38 & 0xF {
                0 => ChatCtrl::Invite {
                    qsy_step: ((body38 >> 4) & 7) as u8,
                    subch: ((body38 >> 7) & 7) as u8,
                },
                1 => ChatCtrl::Accept,
                2 => ChatCtrl::Decline,
                3 => ChatCtrl::End,
                4 => ChatCtrl::Ack {
                    ack_seq: ((body38 >> 4) & 7) as u8,
                    frag_mask: ((body38 >> 7) & 0xFF) as u8,
                },
                _ => return None,
            };
            ChatBody::Ctrl(ctrl)
        }
        1 => ChatBody::Station(StationCard::unpack38(body38)),
        2 => ChatBody::Question { topic: ((body38 >> 3) & 0x1F) as u8, variant: (body38 & 7) as u8 },
        3 => ChatBody::Phrase((body38 & 0xFF) as u8),
        4 => {
            let frag_idx = ((body38 >> 35) & 7) as u8;
            let is_final = ((body38 >> 34) & 1) != 0;
            let char_cnt = ((body38 >> 31) & 7) as u8;
            if char_cnt == 0 || char_cnt > 5 {
                return None;
            }
            let is_name = ((body38 >> 30) & 1) != 0;
            let mut chars = [b' '; 5];
            for (i, slot) in chars.iter_mut().enumerate() {
                let code = ((body38 >> (24 - i * 6)) & 0x3F) as usize;
                *slot = ALPHABET[code];
            }
            ChatBody::LiteralFrag { frag_idx, is_final, char_cnt, is_name, chars }
        }
        5 => ChatBody::Weather(WeatherCard::unpack38(body38)),
        6 => ChatBody::About(AboutCard::unpack38(body38)),
        7 => ChatBody::Rendezvous(RendezvousCard::unpack38(body38)?),
        _ => return None,
    };
    Some(ChatPacket { session_ref, seq, body })
}

/// Normalize ASCII input into the 6-bit RAD950 chat alphabet (uppercase, unsupported chars -> space).
pub fn normalize_literal(src: &[u8], out: &mut [u8; CHAT_LITERAL_MAX]) -> usize {
    let mut n = 0usize;
    for &b in src {
        if n >= CHAT_LITERAL_MAX {
            break;
        }
        let u = b.to_ascii_uppercase();
        out[n] = ALPHABET[encode_char6(u) as usize];
        n += 1;
    }
    while n > 0 && out[n - 1] == b' ' {
        n -= 1;
    }
    n
}

/// Number of 5-char bursts required to transmit a normalized literal message of length `len`.
pub const fn literal_bursts(len: usize) -> u8 {
    if len == 0 {
        0
    } else {
        let clamped = if len > CHAT_LITERAL_MAX { CHAT_LITERAL_MAX } else { len };
        clamped.div_ceil(5) as u8
    }
}

/// Estimated total delivery time (seconds) on the 10 s profile (transmitting on alternating 10 s turns).
pub const fn estimate_airtime_s(bursts: u8) -> u16 {
    if bursts == 0 {
        0
    } else {
        ((bursts as u16) * 2 - 1) * 10 - 1 // e.g. 1 burst -> 9s, 6 bursts -> 109s (1m 49s)
    }
}

/// Encode a normalized literal message into `1..=8` `ChatPacket` fragments.
pub fn encode_literal_frags(
    session_ref: u16,
    seq: u8,
    text: &[u8],
    is_name: bool,
    out: &mut [ChatPacket; CHAT_MAX_FRAGS],
) -> usize {
    let mut norm = [b' '; CHAT_LITERAL_MAX];
    let len = normalize_literal(text, &mut norm);
    let n_frags = literal_bursts(len) as usize;
    for f in 0..n_frags {
        let start = f * 5;
        let cnt = (len - start).min(5);
        let mut chars = [b' '; 5];
        chars[..cnt].copy_from_slice(&norm[start..start + cnt]);
        out[f] = ChatPacket {
            session_ref,
            seq: seq & 7,
            body: ChatBody::LiteralFrag {
                frag_idx: f as u8,
                is_final: f + 1 == n_frags,
                char_cnt: cnt as u8,
                is_name,
                chars,
            },
        };
    }
    n_frags
}

/// Zero-heap reassembler for multi-fragment literal chat messages (`<= 40` chars).
#[derive(Clone, Copy, Debug)]
pub struct LiteralReassembler {
    active_seq: u8,
    received_mask: u8,
    final_idx: Option<u8>,
    chars: [u8; CHAT_LITERAL_MAX],
    lengths: [u8; CHAT_MAX_FRAGS],
}

impl Default for LiteralReassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl LiteralReassembler {
    pub const fn new() -> Self {
        Self {
            active_seq: 0xFF,
            received_mask: 0,
            final_idx: None,
            chars: [b' '; CHAT_LITERAL_MAX],
            lengths: [0; CHAT_MAX_FRAGS],
        }
    }

    pub fn reset(&mut self) {
        self.active_seq = 0xFF;
        self.received_mask = 0;
        self.final_idx = None;
    }

    /// Push one literal fragment; returns `Some((len, frag_mask))` once all fragments `0..=final_idx` arrive.
    pub fn push(&mut self, seq: u8, frag_idx: u8, is_final: bool, char_cnt: u8, chars: [u8; 5]) -> Option<(usize, u8)> {
        if frag_idx as usize >= CHAT_MAX_FRAGS || char_cnt == 0 || char_cnt > 5 {
            return None;
        }
        if self.active_seq != seq {
            self.reset();
            self.active_seq = seq;
        }
        let bit = 1u8 << frag_idx;
        if (self.received_mask & bit) == 0 {
            let base = frag_idx as usize * 5;
            self.chars[base..base + char_cnt as usize].copy_from_slice(&chars[..char_cnt as usize]);
            self.lengths[frag_idx as usize] = char_cnt;
            self.received_mask |= bit;
        }
        if is_final {
            self.final_idx = Some(frag_idx);
        }
        let f_idx = self.final_idx?;
        let need_mask = if f_idx == 7 { 0xFF } else { (1u8 << (f_idx + 1)) - 1 };
        if (self.received_mask & need_mask) == need_mask {
            let mut total = 0usize;
            for i in 0..=(f_idx as usize) {
                total += self.lengths[i] as usize;
            }
            Some((total, self.received_mask))
        } else {
            None
        }
    }

    pub fn bytes(&self, len: usize) -> &[u8] {
        &self.chars[..len.min(CHAT_LITERAL_MAX)]
    }

    pub fn received_mask(&self) -> u8 {
        self.received_mask
    }

    pub fn active_seq(&self) -> u8 {
        self.active_seq
    }
}

fn encode_char6(b: u8) -> u8 {
    let u = b.to_ascii_uppercase();
    for (i, &c) in ALPHABET.iter().enumerate() {
        if c == u {
            return i as u8;
        }
    }
    0 // space
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{Callsign, Frame};

    #[test]
    fn chat_cards_and_literal_round_trip() {
        // 1. Station card round-trip (1 burst)
        let sc = StationCard::DEFAULT_RT950;
        let pkt = ChatPacket { session_ref: 0x3A5, seq: 2, body: ChatBody::Station(sc) };
        let frame = encode_chat_frame(pkt);
        let bits = frame.to_bits78();
        let decoded = Frame::from_bits78(&bits).expect("valid CRC-14");
        assert_eq!(decode_chat_frame(decoded), Some(pkt));
        // Confirm legacy callsign range strictly rejects the out-of-range sender marker.
        assert!(Callsign::from_raw(0x0FC0_0000).is_none());

        // 2. Question, Phrase, Weather, About round-trip (1 burst each)
        for body in [
            ChatBody::Ctrl(ChatCtrl::Invite { qsy_step: 1, subch: 5 }),
            ChatBody::Ctrl(ChatCtrl::Ack { ack_seq: 5, frag_mask: 0x3F }),
            ChatBody::Question { topic: 0, variant: 0 },
            ChatBody::Phrase(1),
            ChatBody::Weather(WeatherCard::DEFAULT),
            ChatBody::Weather(WeatherCard { cond: 0, temp_c: i8::MIN, wind_kph: 127, wind_dir: 0 }),
            ChatBody::About(AboutCard::DEFAULT),
            ChatBody::About(AboutCard { age: 42, year_licensed: 0 }),
            ChatBody::Rendezvous(RendezvousCard::Schedule {
                action: RendezvousAction::Offer,
                days_ahead: 1,
                utc_min: 18 * 60 + 30,
                dial_preset: 1,
                profile: 0,
                subch: 2,
                offer_ref: 3,
            }),
            ChatBody::Rendezvous(RendezvousCard::MoveFreq {
                action: RendezvousAction::Accept,
                band_id: 0,
                dial_offset_10hz: 8200, // 28.082 MHz
                profile: 0,
                turn_mod64: 45,
                subch: 4,
            }),
        ] {
            let p = ChatPacket { session_ref: 42, seq: 5, body };
            let f = Frame::from_bits78(&encode_chat_frame(p).to_bits78()).unwrap();
            assert_eq!(decode_chat_frame(f), Some(p));
        }

        // 3. Literal "HEY MATT HOW HAVE YOU BEEN?" -> 6 bursts, 109 s airtime, exact reassembly
        let msg = b"Hey Matt how have you been?";
        let mut frags = [ChatPacket { session_ref: 0, seq: 0, body: ChatBody::Phrase(0) }; CHAT_MAX_FRAGS];
        let n = encode_literal_frags(77, 3, msg, false, &mut frags);
        assert_eq!(n, 6);
        assert_eq!(estimate_airtime_s(n as u8), 109);
        let mut reasm = LiteralReassembler::new();
        let mut got = None;
        for f in frags[..n].iter().rev() {
            // Reassemble out-of-order!
            let wf = Frame::from_bits78(&encode_chat_frame(*f).to_bits78()).unwrap();
            let dp = decode_chat_frame(wf).unwrap();
            if let ChatBody::LiteralFrag { frag_idx, is_final, char_cnt, chars, .. } = dp.body {
                got = reasm.push(dp.seq, frag_idx, is_final, char_cnt, chars);
            }
        }
        let (len, mask) = got.expect("reassembled");
        assert_eq!(mask, 0x3F);
        assert_eq!(reasm.bytes(len), b"HEY MATT HOW HAVE YOU BEEN?");
    }
}
