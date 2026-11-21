//! Integer logarithm: log2 from count-leading-zeros and a 33-entry table, for levels in decibels without floating
//! point (10 log10 x = 3.0103 log2 x).

/// log2(x) in Q8 (1/256), for x > 0; 0 for 0.
pub fn log2_q8(x: u64) -> i32 {
    if x == 0 {
        return 0;
    }
    let e = 63 - x.leading_zeros() as i32;
    // Five bits below the leading one index a table of log2(1 + j/32), Q8; the next bits interpolate.
    let frac = if e >= 13 { (x >> (e - 13)) as u32 & 0x1FFF } else { ((x << (13 - e)) as u32) & 0x1FFF };
    let j = (frac >> 8) as usize;
    let t = (frac & 0xFF) as i32;
    let lo = LOG2_TABLE[j] as i32;
    let hi = LOG2_TABLE[j + 1] as i32;
    (e << 8) + lo + (((hi - lo) * t) >> 8)
}

/// log2(1 + j/32) in Q8, j = 0..=32.
static LOG2_TABLE: [u16; 33] = log2_table();

const fn log2_table() -> [u16; 33] {
    let mut t = [0u16; 33];
    let mut j = 0;
    while j <= 32 {
        // ln(1 + y) by atanh series, y = j / 32.
        let y = j as f64 / 32.0;
        let u = y / (2.0 + y);
        let mut s = 0.0;
        let mut p = u;
        let mut n = 1;
        while n < 40 {
            s += p / n as f64;
            p *= u * u;
            n += 2;
        }
        let v = 2.0 * s / core::f64::consts::LN_2 * 256.0 + 0.5;
        t[j] = v as u16;
        j += 1;
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The table builder run again at run time gives the flash table: log2(1 + j/32) in Q8, rounded.
    #[test]
    fn table_matches_its_builder() {
        let t = log2_table();
        assert_eq!(t, LOG2_TABLE);
        assert_eq!((t[0], t[16], t[32]), (0, 150, 256)); // log2(1.5) = 0.585 = 149.8 / 256
        for j in 0..=32 {
            let want = (1.0 + j as f64 / 32.0).log2() * 256.0;
            assert!((t[j] as f64 - want).abs() <= 0.5, "j {j}: {} vs {want}", t[j]);
        }
    }
}
