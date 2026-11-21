//! CRCs used by the radio formats in this repository.

/// CRC-16/X.25 (also called CRC-16/IBM-SDLC, HDLC's FCS): polynomial 0x1021 reflected (0x8408), initial 0xFFFF,
/// final XOR 0xFFFF. AX.25 sends it low byte first after the frame.
pub fn crc16_x25(data: &[u8]) -> u16 {
    !crc16_x25_update(0xFFFF, data)
}

/// Running form: start from 0xFFFF, feed bytes, invert at the end.
#[inline]
pub fn crc16_x25_update(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}

/// What [`crc16_x25_update`] leaves after a frame followed by its own correct FCS.
pub const CRC16_X25_GOOD: u16 = 0xF0B8;
