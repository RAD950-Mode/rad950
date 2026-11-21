//! Systematic `(184, 78)` PEG LDPC code (girth 6, zero 4-cycles) with IEEE 802.11n-style bridged dual-diagonal
//! parity ($M = 106$, $K = 78$, $N = 184$).
//!
//! - Systematic message bits `0..78` each have column degree 4 (`MSG_ROWS`).
//! - Parity column `78` ($p_0$) connects to rows `0`, `53`, and `105` (degree 3); parity columns `78 + c` ($c = 1..106$)
//!   connect to rows `c - 1` and `c` (degree 2). Summing all 106 rows cancels every $p_c$ ($c \ge 1$) and leaves $p_0$,
//!   giving exact linear-time systematic encoding from the 312-byte `MSG_ROWS` table.
//! - Decoding uses integer normalized min-sum belief propagation (`scale = 3/4`).

pub const K_BITS: usize = 78;
pub const M_CHECKS: usize = 106;
pub const N_BITS: usize = K_BITS + M_CHECKS; // 184
const MID_ROW: usize = M_CHECKS / 2; // 53
pub const N_EDGES: usize = K_BITS * 4 + 3 + (M_CHECKS - 1) * 2; // 525

/// Row indices (`0..106`) of the four `1`s in each systematic message column `0..78`.
/// Constructed by Progressive Edge Growth (PEG, seed 1) with zero 4-cycles across the full `106 x 184` matrix.
pub static MSG_ROWS: [[u8; 4]; K_BITS] = [
    [17, 44, 79, 97],
    [6, 31, 66, 89],
    [28, 48, 59, 73],
    [11, 38, 56, 84],
    [24, 40, 91, 102],
    [4, 14, 22, 50],
    [61, 69, 81, 92],
    [2, 36, 64, 77],
    [8, 41, 52, 75],
    [13, 33, 70, 99],
    [27, 87, 96, 105],
    [3, 10, 46, 93],
    [19, 74, 83, 100],
    [34, 42, 58, 94],
    [16, 29, 39, 68],
    [21, 32, 62, 86],
    [23, 54, 65, 72],
    [49, 67, 78, 103],
    [7, 12, 63, 104],
    [47, 55, 90, 98],
    [5, 37, 95, 101],
    [15, 35, 82, 88],
    [1, 25, 30, 57],
    [43, 51, 71, 85],
    [20, 57, 76, 80],
    [0, 9, 18, 60],
    [26, 45, 49, 70],
    [20, 46, 53, 102],
    [62, 67, 83, 94],
    [0, 33, 39, 101],
    [4, 27, 34, 55],
    [13, 28, 66, 76],
    [12, 17, 54, 92],
    [30, 37, 47, 51],
    [11, 15, 32, 52],
    [7, 24, 35, 48],
    [64, 68, 71, 90],
    [2, 6, 60, 85],
    [40, 65, 79, 86],
    [10, 23, 29, 99],
    [21, 41, 81, 96],
    [3, 58, 91, 97],
    [16, 25, 42, 61],
    [8, 26, 84, 98],
    [1, 72, 88, 103],
    [5, 44, 77, 105],
    [19, 22, 38, 59],
    [9, 31, 45, 63],
    [18, 56, 75, 93],
    [43, 74, 80, 104],
    [50, 69, 73, 95],
    [14, 78, 87, 100],
    [36, 53, 89, 97],
    [25, 38, 44, 82],
    [12, 30, 73, 86],
    [9, 27, 68, 82],
    [59, 63, 88, 98],
    [2, 47, 70, 75],
    [4, 29, 36, 62],
    [33, 43, 65, 96],
    [14, 26, 58, 72],
    [17, 52, 67, 101],
    [32, 48, 92, 105],
    [7, 18, 42, 50],
    [16, 37, 66, 104],
    [21, 60, 78, 90],
    [31, 41, 54, 100],
    [11, 34, 45, 80],
    [10, 69, 79, 83],
    [1, 13, 55, 81],
    [8, 28, 71, 102],
    [5, 23, 61, 74],
    [39, 51, 77, 94],
    [24, 76, 84, 87],
    [40, 56, 89, 99],
    [15, 49, 57, 64],
    [3, 19, 95, 103],
    [0, 22, 66, 91],
];

struct CheckGraph {
    offsets: [u16; M_CHECKS + 1],
    cols: [u8; N_EDGES],
}

static GRAPH: CheckGraph = build_graph();

const fn build_graph() -> CheckGraph {
    let mut counts = [0u16; M_CHECKS];
    let mut c = 0;
    while c < K_BITS {
        let mut k = 0;
        while k < 4 {
            counts[MSG_ROWS[c][k] as usize] += 1;
            k += 1;
        }
        c += 1;
    }
    counts[0] += 1;
    counts[MID_ROW] += 1;
    counts[M_CHECKS - 1] += 1;
    let mut p = 1;
    while p < M_CHECKS {
        counts[p - 1] += 1;
        counts[p] += 1;
        p += 1;
    }

    let mut offsets = [0u16; M_CHECKS + 1];
    let mut r = 0;
    while r < M_CHECKS {
        offsets[r + 1] = offsets[r] + counts[r];
        r += 1;
    }

    let mut cur = [0u16; M_CHECKS];
    let mut r = 0;
    while r < M_CHECKS {
        cur[r] = offsets[r];
        r += 1;
    }
    let mut cols = [0u8; N_EDGES];

    let mut c = 0;
    while c < K_BITS {
        let mut k = 0;
        while k < 4 {
            let row = MSG_ROWS[c][k] as usize;
            cols[cur[row] as usize] = c as u8;
            cur[row] += 1;
            k += 1;
        }
        c += 1;
    }
    // p_0 at column K_BITS
    let p0_rows = [0usize, MID_ROW, M_CHECKS - 1];
    let mut k = 0;
    while k < 3 {
        let row = p0_rows[k];
        cols[cur[row] as usize] = K_BITS as u8;
        cur[row] += 1;
        k += 1;
    }
    // p_1 .. p_105
    let mut p = 1;
    while p < M_CHECKS {
        let col = (K_BITS + p) as u8;
        let r0 = p - 1;
        let r1 = p;
        cols[cur[r0] as usize] = col;
        cur[r0] += 1;
        cols[cur[r1] as usize] = col;
        cur[r1] += 1;
        p += 1;
    }

    CheckGraph { offsets, cols }
}

/// Encode 78 systematic bits (`0` or `1`) into a 184-bit systematic codeword (`[0..78]` = `msg`, `[78..184]` = parity).
pub fn encode(msg: &[u8; K_BITS]) -> [u8; N_BITS] {
    let mut s = [0u8; M_CHECKS];
    for (c, &b) in msg.iter().enumerate() {
        if (b & 1) != 0 {
            for &r in &MSG_ROWS[c] {
                s[r as usize] ^= 1;
            }
        }
    }

    let mut cw = [0u8; N_BITS];
    cw[..K_BITS].copy_from_slice(msg);

    let mut p0 = 0u8;
    for &sr in &s {
        p0 ^= sr;
    }
    cw[K_BITS] = p0;
    cw[K_BITS + 1] = s[0] ^ p0;
    for c in 2..=MID_ROW {
        cw[K_BITS + c] = s[c - 1] ^ cw[K_BITS + c - 1];
    }
    cw[K_BITS + MID_ROW + 1] = s[MID_ROW] ^ cw[K_BITS + MID_ROW] ^ p0;
    for c in (MID_ROW + 2)..M_CHECKS {
        cw[K_BITS + c] = s[c - 1] ^ cw[K_BITS + c - 1];
    }
    cw
}

/// Check whether all 106 parity equations hold for `cw`.
pub fn check_syndrome(cw: &[u8; N_BITS]) -> bool {
    for r in 0..M_CHECKS {
        let start = GRAPH.offsets[r] as usize;
        let end = GRAPH.offsets[r + 1] as usize;
        let mut x = 0u8;
        for &col in &GRAPH.cols[start..end] {
            x ^= cw[col as usize] & 1;
        }
        if x != 0 {
            return false;
        }
    }
    true
}

/// Normalized min-sum belief propagation decoder (`llr > 0` favors bit `0`, `llr < 0` favors bit `1`) using
/// caller-supplied `post` (`184 x i16`) and `c2v` (`525 x i16`) scratch slices (zero stack scratch).
/// Returns the 78 systematic bits once all 106 parity checks are satisfied.
pub fn decode_bp_with_slices(
    llr: &[i8; N_BITS],
    post: &mut [i16; N_BITS],
    c2v: &mut [i16; N_EDGES],
    max_iter: usize,
) -> Option<[u8; K_BITS]> {
    c2v.fill(0);
    for (i, &v) in llr.iter().enumerate() {
        post[i] = v as i16;
    }

    let mut hard = [0u8; N_BITS];
    for _ in 0..max_iter {
        for (i, &p) in post.iter().enumerate() {
            hard[i] = (p < 0) as u8;
        }
        if check_syndrome(&hard) {
            let mut out = [0u8; K_BITS];
            out.copy_from_slice(&hard[..K_BITS]);
            return Some(out);
        }

        for r in 0..M_CHECKS {
            let start = GRAPH.offsets[r] as usize;
            let end = GRAPH.offsets[r + 1] as usize;
            let deg = end - start;
            // Max row degree in GRAPH is 6.
            let mut v2c = [0i16; 8];
            let mut sign_neg = 0u8;
            let mut min1 = i16::MAX;
            let mut min2 = i16::MAX;
            let mut min1_idx = 0usize;

            for (idx, e) in (start..end).enumerate() {
                let col = GRAPH.cols[e] as usize;
                let m = post[col] - c2v[e];
                v2c[idx] = m;
                if m < 0 {
                    sign_neg ^= 1;
                }
                let am = m.saturating_abs();
                if am < min1 {
                    min2 = min1;
                    min1 = am;
                    min1_idx = idx;
                } else if am < min2 {
                    min2 = am;
                }
            }

            for (idx, e) in (start..end).take(deg).enumerate() {
                let col = GRAPH.cols[e] as usize;
                let neg = sign_neg ^ ((v2c[idx] < 0) as u8);
                let raw_mag = if idx == min1_idx { min2 } else { min1 };
                let mag = ((raw_mag * 3 + 2) >> 2).min(96);
                let new_c2v = if neg != 0 { -mag } else { mag };
                post[col] = (post[col] + (new_c2v - c2v[e])).clamp(-2048, 2047);
                c2v[e] = new_c2v;
            }
        }
    }

    for (i, &p) in post.iter().enumerate() {
        hard[i] = (p < 0) as u8;
    }
    if check_syndrome(&hard) {
        let mut out = [0u8; K_BITS];
        out.copy_from_slice(&hard[..K_BITS]);
        Some(out)
    } else {
        None
    }
}

/// Reusable scratch for normalized min-sum belief propagation (`1,418 bytes`).
pub struct BpScratch {
    c2v: [i16; N_EDGES],
    post: [i16; N_BITS],
}

impl BpScratch {
    pub const fn new() -> Self {
        Self { c2v: [0; N_EDGES], post: [0; N_BITS] }
    }
}

/// Convenience wrapper around [`decode_bp_with_slices`] using a [`BpScratch`] struct.
pub fn decode_bp(llr: &[i8; N_BITS], scratch: &mut BpScratch, max_iter: usize) -> Option<[u8; K_BITS]> {
    decode_bp_with_slices(llr, &mut scratch.post, &mut scratch.c2v, max_iter)
}
