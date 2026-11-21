//! RAD950 Standalone Distress Alert & Precise Location (`docs/rad950-conversation-extension.md`).
//!
//! Uses `Frame::ExtFrag { kind: DISTRESS_EXT_KIND (3), .. }` (`55` bits available):
//! - `bit 54 = 0`: **Alert** (`54` bits):
//!   `28b sender | 15b grid4 (0x7FFF = unknown) | 4b category | 2b loc_src | 5b seq`
//! - `bit 54 = 1`: **Position** (`54` bits):
//!   `21b lat_u (0.0001 deg from -90) | 22b lon_u (0.0001 deg from -180) | 11b assoc_cksum`

use crate::frame::{decode_grid4, Callsign, Frame};

/// Extension `kind` reserved for RAD950 Distress frames inside `Frame::ExtFrag`.
pub const DISTRESS_EXT_KIND: u8 = 3;

/// Sentinel `grid4` value when no grid is available (`15` bits all ones).
pub const GRID4_UNKNOWN: u16 = 0x7FFF;

/// Optional emergency category (`4` bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistressCategory {
    Unspecified = 0,
    Medical = 1,
    Fire = 2,
    Stranded = 3,
    Weather = 4,
    Mechanical = 5,
}

impl DistressCategory {
    pub const COUNT: u8 = 6;

    pub const fn from_u8(v: u8) -> Self {
        match v & 0x0F {
            1 => Self::Medical,
            2 => Self::Fire,
            3 => Self::Stranded,
            4 => Self::Weather,
            5 => Self::Mechanical,
            _ => Self::Unspecified,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Unspecified => "Unspecified",
            Self::Medical => "Medical",
            Self::Fire => "Fire",
            Self::Stranded => "Stranded",
            Self::Weather => "Weather",
            Self::Mechanical => "Mechanical",
        }
    }
}

/// Source of the distress position (`2` bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistressLocSource {
    Unavailable = 0,
    FreshGps = 1,
    Manual = 2,
    LastKnownGps = 3,
}

impl DistressLocSource {
    pub const fn from_u8(v: u8) -> Self {
        match v & 3 {
            1 => Self::FreshGps,
            2 => Self::Manual,
            3 => Self::LastKnownGps,
            _ => Self::Unavailable,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Unavailable => "Grid only",
            Self::FreshGps => "Fresh GPS",
            Self::Manual => "Manual",
            Self::LastKnownGps => "Last known GPS",
        }
    }
}

/// Burst 1: self-contained standalone distress alert (`54` bits).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DistressAlert {
    pub sender: Callsign,
    pub grid4: u16,
    pub category: DistressCategory,
    pub loc_src: DistressLocSource,
    pub seq: u8,
}

impl DistressAlert {
    pub fn pack54(&self) -> u64 {
        let s = (self.sender.raw() as u64) & 0x0FFF_FFFF;
        let g = (self.grid4 as u64) & 0x7FFF;
        let c = (self.category as u64) & 0x0F;
        let l = (self.loc_src as u64) & 0x03;
        let q = (self.seq as u64) & 0x1F;
        (s << 26) | (g << 11) | (c << 7) | (l << 5) | q
    }

    pub fn unpack54(v: u64) -> Option<Self> {
        let sender = Callsign::from_raw(((v >> 26) & 0x0FFF_FFFF) as u32)?;
        let grid4 = ((v >> 11) & 0x7FFF) as u16;
        if grid4 != GRID4_UNKNOWN && grid4 >= 32_400 {
            return None;
        }
        let category = DistressCategory::from_u8(((v >> 7) & 0x0F) as u8);
        let loc_src = DistressLocSource::from_u8(((v >> 5) & 0x03) as u8);
        let seq = (v & 0x1F) as u8;
        Some(Self { sender, grid4, category, loc_src, seq })
    }
}

/// Burst 2: optional linked precise position (`0.0001°` steps + 11-bit association checksum).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DistressPosition {
    /// Latitude in `0.0001°` steps (`-900_000..=+900_000`).
    pub lat_e4: i32,
    /// Longitude in `0.0001°` steps (`-1_800_000..=+1_799_999`; `+180°` normalized to `-180°`).
    pub lon_e4: i32,
    /// 11-bit association checksum binding this burst to the preceding `DistressAlert`.
    pub assoc_cksum: u16,
}

impl DistressPosition {
    /// Normalize coordinates (`+180.0000°` -> `-180.0000°`) and reject out-of-range values.
    pub fn new(lat_e4: i32, mut lon_e4: i32, assoc_cksum: u16) -> Option<Self> {
        if !(-900_000..=900_000).contains(&lat_e4) || !(-1_800_000..=1_800_000).contains(&lon_e4) {
            return None;
        }
        if lon_e4 == 1_800_000 {
            lon_e4 = -1_800_000;
        }
        Some(Self { lat_e4, lon_e4, assoc_cksum: assoc_cksum & 0x7FF })
    }

    pub fn pack54(&self) -> u64 {
        let lat_u = (self.lat_e4 + 900_000).clamp(0, 1_800_000) as u64;
        let lon_u = (self.lon_e4 + 1_800_000).clamp(0, 3_599_999) as u64;
        let ck = (self.assoc_cksum as u64) & 0x7FF;
        (lat_u << 33) | (lon_u << 11) | ck
    }

    pub fn unpack54(v: u64) -> Option<Self> {
        let lat_u = ((v >> 33) & 0x1F_FFFF) as i32;
        let lon_u = ((v >> 11) & 0x3F_FFFF) as i32;
        if lat_u > 1_800_000 || lon_u > 3_599_999 {
            return None;
        }
        let ck = (v & 0x7FF) as u16;
        Some(Self { lat_e4: lat_u - 900_000, lon_e4: lon_u - 1_800_000, assoc_cksum: ck })
    }
}

/// Decoded standalone distress burst.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistressFrame {
    Alert(DistressAlert),
    Position(DistressPosition),
}

/// Compute the 11-bit association checksum over alert payload, alert slot, dial, subchannel, and coordinates.
pub fn distress_assoc_cksum(
    alert: &DistressAlert,
    alert_slot: u32,
    dial_hz10: u32,
    subch: u8,
    lat_e4: i32,
    lon_e4: i32,
) -> u16 {
    let a = alert.pack54();
    let mut h = (a ^ (a >> 27)) as u32;
    h = h.wrapping_mul(0x9E37_79B1) ^ alert_slot.wrapping_mul(0x85EB_CA77);
    h = h.wrapping_mul(0xC2B2_AE3D) ^ dial_hz10;
    h ^= ((subch as u32) << 24) ^ (lat_e4 as u32).wrapping_mul(0x27D4_EB2D) ^ (lon_e4 as u32).wrapping_mul(0x1656_67B1);
    h ^= h >> 16;
    h ^= h >> 8;
    (h & 0x7FF) as u16
}

/// Verify that `(lat_e4, lon_e4)` falls inside the advertised Maidenhead `grid4` square.
pub fn grid4_matches_coords(grid4: u16, lat_e4: i32, lon_e4: i32) -> bool {
    if grid4 == GRID4_UNKNOWN {
        return true;
    }
    let Some(g) = decode_grid4(grid4) else {
        return false;
    };
    let lon_f = (g[0] - b'A') as i32;
    let lat_f = (g[1] - b'A') as i32;
    let lon_s = (g[2] - b'0') as i32;
    let lat_s = (g[3] - b'0') as i32;
    let lon_min = (-180 + lon_f * 20 + lon_s * 2) * 10_000;
    let lon_max = lon_min + 20_000;
    let lat_min = (-90 + lat_f * 10 + lat_s) * 10_000;
    let lat_max = lat_min + 10_000;
    lon_e4 >= lon_min && lon_e4 < lon_max && lat_e4 >= lat_min && lat_e4 < lat_max
}

pub fn encode_distress_frame(df: DistressFrame) -> Frame {
    let bits55 = match df {
        DistressFrame::Alert(a) => a.pack54() & ((1u64 << 54) - 1),
        DistressFrame::Position(p) => (1u64 << 54) | (p.pack54() & ((1u64 << 54) - 1)),
    };
    let idx = ((bits55 >> 53) & 3) as u8;
    let tag = ((bits55 >> 45) & 0xFF) as u8;
    let payload = bits55 & ((1u64 << 45) - 1);
    Frame::ExtFrag { kind: DISTRESS_EXT_KIND, idx, tag, payload }
}

pub fn decode_distress_frame(frame: Frame) -> Option<DistressFrame> {
    let Frame::ExtFrag { kind: DISTRESS_EXT_KIND, idx, tag, payload } = frame else {
        return None;
    };
    let bits55 = (((idx as u64) & 3) << 53) | (((tag as u64) & 0xFF) << 45) | (payload & ((1u64 << 45) - 1));
    let is_pos = ((bits55 >> 54) & 1) != 0;
    let body54 = bits55 & ((1u64 << 54) - 1);
    if is_pos {
        Some(DistressFrame::Position(DistressPosition::unpack54(body54)?))
    } else {
        Some(DistressFrame::Alert(DistressAlert::unpack54(body54)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::encode_grid4;

    #[test]
    fn distress_alert_and_position_round_trip() {
        let sender = Callsign::encode(b"KF8QE").unwrap();
        let grid4 = encode_grid4(b"CM87").unwrap();
        let alert = DistressAlert {
            sender,
            grid4,
            category: DistressCategory::Medical,
            loc_src: DistressLocSource::FreshGps,
            seq: 7,
        };
        let wf = Frame::from_bits78(&encode_distress_frame(DistressFrame::Alert(alert)).to_bits78()).unwrap();
        assert_eq!(decode_distress_frame(wf), Some(DistressFrame::Alert(alert)));

        // San Francisco in CM87: 37.7749 N, -122.4194 W
        let (lat, lon) = (377_749, -1_224_194);
        assert!(grid4_matches_coords(grid4, lat, lon));
        assert!(!grid4_matches_coords(encode_grid4(b"FN31").unwrap(), lat, lon));

        let ck = distress_assoc_cksum(&alert, 100, 2_807_800, 3, lat, lon);
        let pos = DistressPosition::new(lat, lon, ck).unwrap();
        let pf = Frame::from_bits78(&encode_distress_frame(DistressFrame::Position(pos)).to_bits78()).unwrap();
        assert_eq!(decode_distress_frame(pf), Some(DistressFrame::Position(pos)));

        // Normalize +180.0000 -> -180.0000 and reject out-of-range latitude.
        assert_eq!(DistressPosition::new(0, 1_800_000, 0).unwrap().lon_e4, -1_800_000);
        assert!(DistressPosition::new(900_001, 0, 0).is_none());
    }
}
