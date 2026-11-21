//! Integer FFT: an in-place complex radix-2 FFT on 16-bit values with block floating point (a stage halves
//! everything only when a value could overflow, and the shifts are counted), and the post-processing that turns it
//! into the spectrum of a real frame of twice the length.

use crate::arm::{load2, pack, smuadx, smusd, store2, unpack};
use crate::trig::cos_sin;

/// In-place complex FFT of `m` points (interleaved re, im in `z`, `m` a power of two dividing 4096). Returns the
/// number of halvings applied: the true result is `z * 2^shift`.
///
/// On an ARMv7E-M target this is [`cfft_packed`] (the M4's dual 16-bit multiplies), elsewhere [`cfft_plain`]; the
/// two give bit-identical results.
pub fn cfft(z: &mut [i16], m: usize) -> u32 {
    #[cfg(any(udsp_dsp, feature = "packed"))]
    {
        cfft_packed(z, m)
    }
    #[cfg(not(any(udsp_dsp, feature = "packed")))]
    {
        cfft_plain(z, m)
    }
}

/// Bit-reversed reordering, shared by both kernels.
fn reorder(z: &mut [i16], m: usize) {
    let mut j = 0;
    for i in 0..m {
        if i < j {
            z.swap(2 * i, 2 * j);
            z.swap(2 * i + 1, 2 * j + 1);
        }
        let mut bit = m >> 1;
        while bit > 0 && j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
    }
}

/// The halving rule both kernels share: halve a stage's output when any part could pass 2^15 / (1 + sqrt 2).
#[inline(always)]
fn stage_shift(z: &[i16]) -> u32 {
    let peak = z.iter().map(|v| (*v as i32).abs()).max().unwrap_or(0);
    if peak >= 13_572 {
        1
    } else {
        0
    }
}

/// [`stage_shift`] for the packed kernel: a word (two values) per load, and done at the first value with
/// |v| >= 13,572 (v + 13,571 outside 0..=27,142 as an unsigned number).
#[inline(always)]
fn stage_shift_packed(z: &[i16], m: usize) -> u32 {
    let mut i = 0;
    while i < 2 * m {
        // SAFETY: i + 1 < 2m <= z.len() (the caller checked).
        let (a, b) = unpack(unsafe { load2(z, i) });
        if (a + 13_571) as u32 > 27_142 || (b + 13_571) as u32 > 27_142 {
            return 1;
        }
        i += 2;
    }
    0
}

/// The reference kernel: one butterfly at a time, each twiddle looked up where it's used.
#[doc(hidden)]
pub fn cfft_plain(z: &mut [i16], m: usize) -> u32 {
    reorder(z, m);
    let mut shift = 0;
    let mut len = 2;
    while len <= m {
        // A butterfly adds a rotated value to another: a part can grow by 1 + sqrt 2. Halve the whole block first
        // if anything is past 2^15 / (1 + sqrt 2).
        let s = stage_shift(&z[..2 * m]);
        shift += s;
        let half = len / 2;
        let step = 4096 / len;
        let mut start = 0;
        while start < m {
            for k in 0..half {
                let (wr, wi) = cos_sin(4096 - k * step); // e^{-2 pi i k / len}
                let a = 2 * (start + k);
                let b = 2 * (start + k + half);
                let (zr, zi) = (z[b] as i32, z[b + 1] as i32);
                let br = (zr * wr - zi * wi + (1 << 14)) >> 15;
                let bi = (zr * wi + zi * wr + (1 << 14)) >> 15;
                let (ar, ai) = (z[a] as i32, z[a + 1] as i32);
                z[a] = ((ar + br) >> s) as i16;
                z[a + 1] = ((ai + bi) >> s) as i16;
                z[b] = ((ar - br) >> s) as i16;
                z[b + 1] = ((ai - bi) >> s) as i16;
            }
            start += len;
        }
        len <<= 1;
    }
    shift
}

/// The M4 kernel: each (re, im) pair is one 32-bit load and store, the complex multiply is two dual 16-bit
/// multiplies (SMUSD for the real part, SMUADX for the imaginary), and each twiddle is packed once per stage and
/// used for every group (butterflies within a stage are independent, so the order doesn't change the result). The
/// halving test reads the block a word (two values) at a time and stops at the first large value. No bounds checks
/// in the loop. Bit-identical to
/// [`cfft_plain`]: the products and sums are the same 32-bit values; only the a +/- b and the halving stay in 32
/// bits, since a rotated part can reach sqrt 2 x 2^15.
#[doc(hidden)]
pub fn cfft_packed(z: &mut [i16], m: usize) -> u32 {
    assert!(m.is_power_of_two() && m <= 4096 && z.len() >= 2 * m);
    reorder(z, m);
    let mut shift = 0;
    let mut len = 2;
    while len <= m {
        let s = stage_shift_packed(z, m);
        shift += s;
        let half = len / 2;
        let step = 4096 / len;
        for k in 0..half {
            let (wr, wi) = cos_sin(4096 - k * step); // e^{-2 pi i k / len}, both within +-32767
            let w = pack(wr as i16, wi as i16);
            let mut a = 2 * k;
            while a < 2 * m {
                let b = a + len; // 2 (start + k + half)
                // SAFETY: a < b + 1 < 2m <= z.len(): start + k + half < m for every group.
                let (za, zb) = unsafe { (load2(z, a), load2(z, b)) };
                let br = (smusd(zb, w) + (1 << 14)) >> 15;
                let bi = (smuadx(zb, w) + (1 << 14)) >> 15;
                let (ar, ai) = unpack(za);
                unsafe {
                    store2(z, a, pack(((ar + br) >> s) as i16, ((ai + bi) >> s) as i16));
                    store2(z, b, pack(((ar - br) >> s) as i16, ((ai - bi) >> s) as i16));
                }
                a += 2 * len;
            }
        }
        len <<= 1;
    }
    shift
}

/// After [`cfft`] of a real frame packed as `z[k] = x[2k] + i x[2k+1]` (m = N/2 points), the power `|X[k]|^2` of real
/// bins k = first, first + stride, ... (`count` of them, each < N/2), passed to `out` in order, in the FFT's units.
pub fn real_power(z: &[i16], m: usize, first: usize, stride: usize, count: usize, mut out: impl FnMut(usize, u64)) {
    let n = 2 * m;
    let per = 4096 / n; // table steps per bin
    for i in 0..count {
        let k = first + i * stride;
        let a = 2 * (k % m);
        let b = 2 * ((m - k % m) % m);
        let (zr, zi) = (z[a] as i32, z[a + 1] as i32);
        let (cr, ci) = (z[b] as i32, -(z[b + 1] as i32)); // conj(Z[m - k])
        let (er, ei) = (zr + cr, zi + ci);
        let (or, oi) = (zr - cr, zi - ci);
        let (wr, wi) = cos_sin(4096 - (k * per) % 4096); // W^k = e^{-2 pi i k / N}
        let tr = (wr * or - wi * oi) >> 15;
        let ti = (wr * oi + wi * or) >> 15;
        let (xr, xi) = ((er + ti) as i64, (ei - tr) as i64); // 2 X[k]
        out(k, ((xr * xr + xi * xi) >> 2) as u64);
    }
}

