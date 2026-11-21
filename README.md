# RAD950

Cross-platform weak-signal 4-FSK digital mode (`RAD950` 10 s & `RAD950-60` 1-minute) — protocol specification, `#![no_std]` embedded DSP crate (`urad950`), and cross-platform desktop transceiver (`rad950` for macOS, Linux, and Windows).

## Workspace Layout

- `docs/rad950-spec.md` — Authoritative RAD950 v1 & RAD950-60 specification + Golden Test Vectors
- `crates/udsp` — `#![no_std]` fixed-point DSP primitives
- `crates/urad950` — `#![no_std]` RAD950 wire format, LDPC(184, 78), `Receiver` (10 s) and `Receiver60` (60 s) suitable for ARM Cortex-M4F handheld firmwares (e.g. Radtel RT-950 Pro)
- `crates/rad950-node` — Shared desktop node engine (3-slot QSO state machine, Hunter auto-answer, Hourly UTC Rendezvous with adaptive congestion backoff, ADIF log, live spotting, `cpal` soundcard audio, Hamlib `rigctld` CAT/PTT, and direct HackRF One backend)
- `crates/rad950-app` — Single cross-platform binary (`rad950`): native `egui` GUI by default, or `ratatui` TUI with `--text`

## Quick Start

```bash
# List soundcard devices
cargo run --release -p rad950-app -- --list-audio

# Launch native GUI (soundcard + Hamlib rigctld)
cargo run --release -p rad950-app -- \
  --call NQ6N --grid EN61 --band 17m \
  --rigctld 127.0.0.1:4532 --dry-tx

# Launch cross-platform terminal TUI (--text)
cargo run --release -p rad950-app -- --text \
  --call NQ6N --grid EN61 --band 17m \
  --backend hackrf --dry-tx
```
