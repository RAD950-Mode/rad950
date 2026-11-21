# RAD-950

Cross-platform weak-signal 4-FSK digital mode (`RAD950` 10 s & `RAD950-60` 1-minute) — protocol specification, `#![no_std]` embedded DSP & modem crate (`urad950`), and standalone desktop transceiver (`rad950` for macOS, Windows, and Linux).

- **Website & Live Spots:** https://rad950.org
- **Technical Documentation & Spec:** https://rad950.org/docs
- **RT-950 Pro Web USB Flasher (Console):** https://rad950.org/console
- **Prebuilt Releases:** https://github.com/RAD950-Mode/rad950/releases/latest

## Workspace Layout

- `docs/rad950-spec.md` — Authoritative RAD950 v1 & RAD950-60 specification + Golden Test Vectors
- `crates/urad950` — `#![no_std]` clean-room fixed-point DSP, wire format, LDPC(184, 78), `Receiver` (10 s) and `Receiver60` (60 s) suitable for ARM Cortex-M4F handhelds and desktop transceivers
- `crates/rad950-node` — Shared desktop node engine (3-slot QSO state machine, Hunter auto-answer, Hourly UTC Rendezvous, ADIF log, `ALL.TXT`, live spotting & cloud QSO sync, alert chimes, `cpal` soundcard audio, Hamlib `rigctld` CAT/PTT, native TCI v1.8 SDR backend, and direct HackRF One backend)
- `crates/rad950-app` — Single cross-platform binary (`rad950`): native `egui` GUI with live panafall by default, or `ratatui` TUI with `--text`

## Quick Start

Download a prebuilt binary from [Releases](https://github.com/RAD950-Mode/rad950/releases/latest), or build from source with Rust:

```bash
# List soundcard devices
cargo run --release -p rad950-app -- --list-audio

# Launch native GUI (soundcard + Hamlib rigctld)
cargo run --release -p rad950-app -- \
  --call W1AW --grid FN31 --band 17m \
  --rigctld 127.0.0.1:4532 --dry-tx

# Launch native GUI over TCI v1.8 (ExpertSDR / Thetis / TCI bridge)
cargo run --release -p rad950-app -- \
  --tci 127.0.0.1:50001 --call W1AW --grid FN31 --band 17m --dry-tx

# Launch terminal TUI (--text)
cargo run --release -p rad950-app -- --text \
  --tci 127.0.0.1:50001 --call W1AW --grid FN31 --band 17m --dry-tx
```

Settings saved from the GUI **Settings** tab persist automatically to:
- **macOS:** `~/Library/Application Support/rad950/config.toml`
- **Linux:** `~/.config/rad950/config.toml`
- **Windows:** `%APPDATA%\rad950\config.toml`

## License

All source code in this repository (`urad950`, `rad950-node`, `rad950-app`, and `docs/rad950-spec.md`) is licensed under the **MIT License** (see [LICENSE](LICENSE)).

Prebuilt Radtel RT-950 Pro firmware images (`.ffw` / `.btf`) attached to GitHub Releases are built on Ferrite firmware and are distributed for noncommercial amateur radio use under the [PolyForm Noncommercial License 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0):
> Required Notice: Copyright 2026 Alex Harvey (https://ferritefw.dev)
