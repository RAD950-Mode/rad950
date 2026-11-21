//! # udsp
//!
//! Integer DSP building blocks for microcontrollers, shared by the decoders in this repository. `no_std`, no
//! allocation, no floating point at run time; tables are built at compile time and live in flash.
//!
//! - [`trig`]: one quarter-wave sine table for every twiddle, window and oscillator; `const` sin/cos.
//! - [`fft`]: 16-bit radix-2 FFT with block floating point; real-frame spectrum.
//! - [`log`]: integer log2 for decibel levels.
//! - [`mix`]: mixer and decimating FIR filter for complex baseband (compile-time low-pass design).
//! - [`fir`]: decimating FIR filter for real signals.
//! - [`goertzel`]: single-frequency power over a block (tone detection).
//! - [`crc`]: CRC-16/X.25 (the AX.25 and HDLC frame check).
//! - [`arm`]: the Cortex-M4 DSP instructions behind the packed FFT and mixer filter, and a cycle counter.
//!
//! On an ARMv7E-M target (thumbv7em: Cortex-M4/M7) the FFT and the mixer's filter ([`mix::FrontEnd`]) use the M4's
//! dual 16-bit multiplies, selected by the build script; everywhere else they are plain Rust. Both give
//! bit-identical results. The real-signal FIR ([`fir::Decimator`]) stays plain: LLVM's ParallelDSP pass already
//! turns it into SMLAD, unrolled, which beats a hand-written loop. Goertzel's recurrence is one 32 x 32 -> 64
//! multiply per sample (SMULL) with no 16-bit pairs to pack, so it stays as it is. Measured in
//! `tools/check-m4` and the decoders' cpu-bench tools. Features: `plain` keeps the plain kernels on the target (to
//! compare); `packed` runs the packed kernels on any target with emulated instructions (to test them on the PC).

#![no_std]
#![allow(clippy::needless_range_loop)] // indexed loops read better in DSP code

pub mod arm;
pub mod crc;
pub mod fft;
pub mod fir;
pub mod goertzel;
pub mod log;
pub mod mix;
pub mod trig;
