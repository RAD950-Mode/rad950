//! The Cortex-M4 DSP instructions the packed FFT and mixer filter use, with their exact ARM semantics. On an ARMv7E-M target
//! (`cfg(udsp_dsp)`, set by the build script) each is the instruction itself; everywhere else it is plain Rust that
//! computes the same 32-bit result, so the packed kernels run (and are tested against the plain ones) on the PC.
//!
//! A packed word holds two signed 16-bit halves: `lo` in bits 0-15, `hi` in bits 16-31, which is how a
//! little-endian `[i16; 2]` such as an interleaved (re, im) pair reads as a `u32`. None of these saturate: the
//! sums wrap modulo 2^32 (the instructions only set the Q flag), which is what the plain kernels' `i32` sums do.

#[inline(always)]
fn lo(x: u32) -> i32 {
    x as u16 as i16 as i32
}

#[inline(always)]
fn hi(x: u32) -> i32 {
    (x >> 16) as u16 as i16 as i32
}

/// Two halves into one word (PKHBT).
#[inline(always)]
pub fn pack(lo: i16, hi: i16) -> u32 {
    lo as u16 as u32 | (hi as u16 as u32) << 16
}

/// SMUSD: `lo(x) * lo(y) - hi(x) * hi(y)`.
#[inline(always)]
pub fn smusd(x: u32, y: u32) -> i32 {
    #[cfg(udsp_dsp)]
    {
        let r: i32;
        // SAFETY: a register-only arithmetic instruction; no memory, flags or stack touched.
        unsafe { core::arch::asm!("smusd {r}, {x}, {y}", r = lateout(reg) r, x = in(reg) x, y = in(reg) y, options(pure, nomem, nostack, preserves_flags)) };
        r
    }
    #[cfg(not(udsp_dsp))]
    {
        (lo(x) * lo(y)).wrapping_sub(hi(x) * hi(y))
    }
}

/// SMUADX: `lo(x) * hi(y) + hi(x) * lo(y)`.
#[inline(always)]
pub fn smuadx(x: u32, y: u32) -> i32 {
    #[cfg(udsp_dsp)]
    {
        let r: i32;
        // SAFETY: as `smusd`. (SMUADX can set the Q flag, which nothing here reads.)
        unsafe { core::arch::asm!("smuadx {r}, {x}, {y}", r = lateout(reg) r, x = in(reg) x, y = in(reg) y, options(pure, nomem, nostack)) };
        r
    }
    #[cfg(not(udsp_dsp))]
    {
        (lo(x) * hi(y)).wrapping_add(hi(x) * lo(y))
    }
}


/// The halves of a packed word, sign-extended.
#[inline(always)]
pub fn unpack(x: u32) -> (i32, i32) {
    (lo(x), hi(x))
}

/// Reads `s[i]` and `s[i + 1]` as one word (an unaligned 32-bit load on the M4).
///
/// # Safety
/// `i + 1 < s.len()`.
#[inline(always)]
pub unsafe fn load2(s: &[i16], i: usize) -> u32 {
    debug_assert!(i + 1 < s.len());
    (s.as_ptr().add(i) as *const u32).read_unaligned()
}

/// Writes one word to `s[i]` and `s[i + 1]`.
///
/// # Safety
/// `i + 1 < s.len()`.
#[inline(always)]
pub unsafe fn store2(s: &mut [i16], i: usize, v: u32) {
    debug_assert!(i + 1 < s.len());
    (s.as_mut_ptr().add(i) as *mut u32).write_unaligned(v)
}




/// The complex dot product `sum v[k] * h[k]` (real taps), as (re, im): two taps per step, the pair's real parts and
/// imaginary parts each regrouped into one word (PKHBT, PKHTB) and multiplied by the pair of taps (SMLAD). Wins
/// inside the mixer's filter, where the compiler otherwise keeps one multiply per instruction (tools/check-m4 and
/// the uft48/uwspr cpu-bench counts).
#[inline(always)]
pub fn cdot(v: &[[i16; 2]], h: &[i16]) -> (i32, i32) {
    let n = v.len().min(h.len());
    let flat = v.as_flattened();
    let (mut re, mut im) = (0i32, 0i32);
    let mut k = 0;
    while k + 1 < n {
        // SAFETY: k + 1 < n: v[k], v[k + 1] (flat 2k .. 2k + 3) and h[k], h[k + 1] are in range.
        let (x0, x1, hh) = unsafe { (load2(flat, 2 * k), load2(flat, 2 * k + 2), load2(h, k)) };
        re = smlad(x0 & 0xFFFF | x1 << 16, hh, re);
        im = smlad(x0 >> 16 | x1 & 0xFFFF_0000, hh, im);
        k += 2;
    }
    if k < n {
        re = re.wrapping_add(v[k][0] as i32 * h[k] as i32);
        im = im.wrapping_add(v[k][1] as i32 * h[k] as i32);
    }
    (re, im)
}

/// [`cdot`] one tap at a time: the plain kernel, and the tests' reference.
#[inline(always)]
pub fn cdot_plain(v: &[[i16; 2]], h: &[i16]) -> (i32, i32) {
    let (mut re, mut im) = (0i32, 0i32);
    for (x, &t) in v.iter().zip(h) {
        re = re.wrapping_add(x[0] as i32 * t as i32);
        im = im.wrapping_add(x[1] as i32 * t as i32);
    }
    (re, im)
}

/// SMLAD: `acc + lo(x) * lo(y) + hi(x) * hi(y)`.
#[inline(always)]
pub fn smlad(x: u32, y: u32, acc: i32) -> i32 {
    #[cfg(udsp_dsp)]
    {
        let r: i32;
        // SAFETY: as `smuadx`.
        unsafe { core::arch::asm!("smlad {r}, {x}, {y}, {a}", r = lateout(reg) r, x = in(reg) x, y = in(reg) y, a = in(reg) acc, options(pure, nomem, nostack)) };
        r
    }
    #[cfg(not(udsp_dsp))]
    {
        acc.wrapping_add(lo(x) * lo(y)).wrapping_add(hi(x) * hi(y))
    }
}

/// The DWT cycle counter, for timing a kernel on the radio: `cycles(|| ...)` returns the cycles it took (wraps
/// after 2^32). Enables the counter on first use (DEMCR.TRCENA, DWT_CTRL.CYCCNTENA). Cortex-M3/M4/M7 only; there
/// is no counter on the PC, where it returns 0.
pub fn cycles(f: impl FnOnce()) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;
        const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
        const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;
        // SAFETY: the Cortex-M debug block's documented registers; setting the enable bits has no other effect.
        unsafe {
            DEMCR.write_volatile(DEMCR.read_volatile() | 1 << 24);
            DWT_CTRL.write_volatile(DWT_CTRL.read_volatile() | 1);
            let t0 = DWT_CYCCNT.read_volatile();
            f();
            DWT_CYCCNT.read_volatile().wrapping_sub(t0)
        }
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        f();
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The plain complex dot product against 64-bit sums, and its 32-bit wrap at the extremes (as SMLAD's).
    #[test]
    fn cdot_plain_sums() {
        let v = [[1000i16, -2000], [-32768, 32767], [7, 0], [0, -1]];
        let h = [3i16, -4, 32767, 9];
        let (mut re, mut im) = (0i64, 0i64);
        for (x, &t) in v.iter().zip(&h) {
            re += x[0] as i64 * t as i64;
            im += x[1] as i64 * t as i64;
        }
        assert_eq!(cdot_plain(&v, &h), (re as i32, im as i32));
        // Shorter taps than samples: only the pairs that exist count.
        assert_eq!(cdot_plain(&v, &h[..1]), (3000, -6000));
        assert_eq!(cdot_plain(&[], &h), (0, 0));
        // Wrap, not saturate: 3 x (-32768 x -32768) = 3 x 2^30 = 2^31 + 2^30, which wraps negative.
        let big = [[-32768i16, 0]; 3];
        assert_eq!(cdot_plain(&big, &[-32768; 3]).0, (3i64 << 30) as i32);
    }
}
