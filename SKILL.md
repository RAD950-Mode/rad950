---
name: rad950
description: Complete operating, setup, clock-sync, firmware, desktop app, and rad950.org guide for the RAD-950 / RAD950-60 weak-signal HF 4-FSK amateur radio mode on Radtel RT-950 Pro (Ferrite firmware) and desktop transceivers.
---

# RAD-950 Weak-Signal HF Digital Mode — AI Assistant Skill & Operator Guide

**Canonical URL:** `https://rad950.org/SKILL.md`
**Website:** `https://rad950.org`
**GitHub Organization:** `https://github.com/RAD950-Mode`
- Desktop & Core DSP (`urad950`, `rad950-node`, `rad950-app`): `https://github.com/RAD950-Mode/rad950`
- RT-950 Pro Firmware (Ferrite `sol950` branch): `https://github.com/RAD950-Mode/ferrite`
- Website & Cloud Relay: `https://github.com/RAD950-Mode/rad950.org`

Use this guide when helping an amateur radio operator understand, install, configure, operate, or troubleshoot **RAD-950** (and **RAD950-60**) on either a **Radtel RT-950 Pro handheld** running Ferrite firmware or the **RAD-950 Desktop App** (Windows, macOS, Linux).

---

## 1. Mode & Protocol Overview

RAD-950 is an open weak-signal 4-FSK digital mode designed so a $50–$80 handheld (Radtel RT-950 Pro running Ferrite firmware) can make real HF QSOs on 17m, 15m, 12m, and 10m with other handhelds or standard desktop HF rigs.

- **Modulation:** Continuous-phase 4-FSK, **12.5 baud** (80 ms/symbol), **50 Hz** tone spacing (150 Hz total occupied bandwidth per signal).
- **FEC & Framing:** 72-bit structured payload (callsigns, 4-char Maidenhead grid, SNR report, QSO state, or Rendezvous/Chat), LDPC forward error correction, CRC-12, 110 symbols total (**8.80 s** burst duration).
- **Slot Timing:**
  - **RAD950 (standard):** **10-second UTC slots** (`:00, :10, :20, :30, :40, :50`). Bursts start at **UTC + 0.40 s** and finish at **UTC + 9.20 s**.
  - **RAD950-60 (ultra-weak-signal):** **60-second UTC minutes** (`:00`–`:49` = 5 coherent repeats of the same 10 s burst folded together for ~+7 dB extra sensitivity; `:50`–`:59` = PA cooldown & decode turnaround).
- **Audio Subchannels (8 channels in USB passband):**
  - `Subch 0`: 600 Hz | `1`: 825 Hz | `2`: 1050 Hz | `3`: 1275 Hz | `4`: **1500 Hz** (default) | `5`: 1725 Hz | `6`: 1950 Hz | `7`: 2175 Hz (225 Hz spacing).
  - Decoder passband floor is **~586 Hz**. Signals shifted below ~586 Hz (e.g., by an uncalibrated BFO) will not decode.
- **Calling Frequencies (USB Dial):**
  - **17m:** `18.104 MHz`
  - **15m:** `21.140 MHz`
  - **12m:** `24.920 MHz`
  - **10m:** `28.180 MHz`
- **Standard QSO Sequence:** `CQ CALL GRID` → `CALL1 CALL2 GRID` → `CALL2 CALL1 ±dB` → `CALL1 CALL2 R±dB` → `CALL2 CALL1 RRR` → `CALL1 CALL2 73` (automatically logged to ADIF / on-radio logbook on completion).

---

## 2. Why Accurate Clocking Is Critical (and How to Get It)

Because RAD-950 uses synchronized 10-second UTC slots with a nominal **+400 ms** burst start, the receiver searches a window of roughly **±0.5 seconds** around the expected start (`start_sym = 1..=9`, i.e., `+80 ms` to `+720 ms` relative to slot start).

- **If your clock is off by > ~0.5 s:** You will see signals on the waterfall/panafall, **but get zero decodes**, and stations will not decode your transmissions.
- **Desktop (Windows / macOS / Linux):**
  - Ensure OS time sync (NTP) is enabled and accurate to `< 100 ms`.
  - On Windows, built-in `w32tm` can drift by 1–2 seconds; recommend **Meinberg NTP** or **Dimension 4** (or running `w32tm /resync`) if decodes fail despite visible signals.
  - On Linux/macOS, `chrony` / `systemd-timesyncd` / Apple NTP normally stay within `< 20 ms`.
- **Radtel RT-950 Pro (3 Standalone On-Radio Clock Sources + USB NTP):**
  1. **GPS (`GPS` in Setup / `Auto`):** Built-in GPS receiver locks UTC to millisecond accuracy and automatically calibrates the MCU's 32.768 kHz crystal so time stays accurate in **crystal-calibrated holdover** even indoors after losing GPS fix.
  2. **OTA Slot Sync (`OTA CLK`):** When indoors without GPS, RAD-950 can lock its slot clock directly from the arrival timing of any decoded over-the-air RAD-950 signal (`utc=1` via OTA).
  3. **WWV / CHU Sync (`WWV CLK`):** Tunes the SI4732 to 10.000 MHz WWV (or CHU) using a crystal-offset-immune square-law AM envelope demodulator to lock the 1-second tick automatically, then retunes back to the active RAD-950 band.
  4. **USB Link NTP Sync:** Connecting the radio via Ferrite Web Console (`https://rad950.org` Console tab), `ferrite_link.py`, or `rad950 --backend rt950` automatically pushes host NTP UTC to the radio (`CLOCK` frame `0x63`).
- **TX Safety Rule:** The RT-950 Pro intentionally refuses to transmit (`Need UTC to TX`) until it has a valid UTC clock from one of the sources above.

---

## 3. Setting Up & Using RAD-950 on the Radtel RT-950 Pro

### 3.1 Flashing Ferrite Firmware (Web Console — No CLI Required)
1. Use a Chromium-based browser (Chrome, Edge, Brave) and a standard **Kenwood K1 / CH340 USB programming cable** plugged into the RT-950 Pro.
2. Go to **`https://rad950.org/on-the-air/get-on-the-air?tab=handheld`** (or the **Firmware / Web Console** tab).
3. **First time only — Take a full 4 MB SPI Flash Backup:** The Web Console guides you to flash `ferrite-reader.btf`, dump your radio's unique factory calibration + stock firmware backup (`.fbin`), and save it safely. **Never lose this backup** — it holds your radio's per-unit TX PA calibration and lets you return to stock at any time.
4. Flash the latest **`ferrite-tx.ffw`** directly from the Web Console (served from `https://rad950.org/api/firmware/latest`, which tracks the latest `RAD950-Mode/rad950` GitHub Release).
5. *CLI alternative:* `python3 tools/flash.py PORT ferrite-tx.ffw --backup my-backup.fbin --dev-unsigned` (from `RAD950-Mode/ferrite`).

### 3.2 Operating RAD-950 on the Handheld
- **Open RAD-950:** Press **`MENU` → `0` → `3` → `7`** (Tools → RAD950).
- **Operate Screen Layout:** Desktop-style vertical **panafall** (live spectrum + downward-scrolling waterfall) with **Band Activity** and **My Activity** panes.
- **Controls on `rad950.operate`:**
  - **Top Knob:** Switch HF calling bands (`17m` / `15m` / `12m` / `10m`).
  - **`1`:** Start calling **CQ** on the next 10 s UTC slot (enters Active QSO screen).
  - **`Up` / `Down` + `OK`:** Select a heard station in Band Activity and **Answer** (auto-sequences the full QSO).
  - **`0`:** **Monitor / Halt TX** immediately.
  - **`*` (Star):** Open **RAD950 Setup** (Callsign, Grid, Band, Subchannel, TX Power Low/Mid/High, **`RX corr`** BFO trim, `RAD950-60` toggle, Clock sync).
  - **`#` (Hash):** Open **RAD950 Menu** (Logbook, Stats, Worked Grids, Rendezvous, Chat, Distress, Cooperative RX).
  - **`2` / `3`:** Quick shortcuts to **Log** and **Stats**.
  - **`EXIT`:** Leave RAD-950 and return to normal VFO/channel operation.

### 3.3 Critical Handheld Calibration: SI4732 `RX corr` (BFO Offset)
Every SI4732 HF receiver chip has a small crystal tolerance on 10m–17m (commonly **-500 Hz to -800 Hz** on 17m/10m).
- **Symptom:** A station transmitting on Subch 4 (`1500 Hz`) appears at `~750 Hz` on your panafall, and stations on Subch 0–3 (`600–1275 Hz`) fall below the `586 Hz` decode floor and cannot be decoded.
- **Fix:** Open **RAD950 Setup (`*`)** and adjust **`RX corr`** (e.g., `+725 Hz` or `+785 Hz`) until a known `1500 Hz` signal lands right at `1500 Hz` on the panafall.

### 3.4 Bonus Mode: RTCW ("Radtel CW")
Ferrite also includes **RTCW** (`MENU 0 3 8` — Tools → RTCW) for standard HF CW on 17m/15m/12m/10m with a built-in iambic keyer (using `PF1`/`PF2` side buttons or a passive K1 jack paddle), `DAC2`-shaped semi-break-in TX, sidetone, and Goertzel CW decoder.

---

## 4. Setting Up & Using the RAD-950 Desktop App

Download prebuilt binaries for **Windows x64**, **macOS (Apple Silicon & Intel)**, and **Linux x64** from:
- **`https://rad950.org`** (Downloads section) or **`https://github.com/RAD950-Mode/rad950/releases/latest`**

The single binary (`rad950` / `rad950.exe`) launches the **Graphical Desktop UI** by default, or pass `--text` for the **Terminal TUI** (ideal for headless Linux/Raspberry Pi or SSH/tmux stations).

### 4.1 Supported Radio Backends (`--backend`)
1. **Sound Card + Hamlib (`--backend audio`, default):**
   - Works with virtually any HF transceiver (IC-7300, FT-891, FT-991A, KX2/KX3, QMX, Xiegu G90, etc.) set to **USB / USB-DATA** with AGC fast/off and no narrow/notch filters.
   - **Audio selection:** Choose Input and Output sound cards in **Settings** (GUI dropdowns) or via CLI (`--list-audio`, `--audio-in "USB Audio"`, `--audio-out "USB Audio"`, `--tx-audio-gain 0.35`).
   - **Hamlib CAT & PTT (`rigctld`):**
     - *External `rigctld`:* Set `rigctld` address to `127.0.0.1:4532` (works with WSJT-X/Hamlib `rigctld` on Windows, macOS, and Linux).
     - *Auto-spawn `rigctld`:* Set Rig Model ID (e.g., `3073` for IC-7300), Serial Port (e.g., `COM3` or `/dev/ttyUSB0`), and Baud Rate in Settings (requires `rigctld` on `PATH`).
2. **TCI WebSocket SDR (`--backend tci --tci 127.0.0.1:50001`):**
   - Direct digital IQ/audio + CAT + PTT over WebSocket for ExpertSDR / Thetis / SunSDR / Hermes Lite 2 / SDR Console.
3. **HackRF One (`--backend hackrf`):**
   - Direct RF TX/RX via `hackrf_transfer` (`--hackrf-serial`, `--lna`, `--vga`, `--tx-gain`, `--ppm`). Great for lab testing or QRP beacons with proper bandpass filtering.
4. **USB-Connected Radtel RT-950 Pro (`--backend rt950 --rt950-port <PORT>`):**
   - Connects directly to an RT-950 Pro running Ferrite (`0.9.3` build `6017913`+) over the CH340 USB programming cable.
   - Automatically opens RAD-950 mode (`MENU 0 3 7`), syncs the radio's UTC clock from NTP, upgrades the serial link to **230,400 baud** for lossless 12 kHz G.711 µ-law RX audio streaming into the desktop decoder + panafall, switches bands via remote knob steps, and keys immediate 110-symbol BK4829 4-FSK TX bursts via Link command `RAD950_TX` (`0x65`).
   - Lets you operate your handheld from the desktop app **or remotely from any browser via Web Remote**.

### 4.2 Safety & Automation Features
- **Dry-TX Mode (`--dry-tx` / `d` key):** Defaults to **ON** on fresh setups so nothing transmits RF until you explicitly switch to Live TX (`--live-tx` or toggle Dry-TX off).
- **TX Watchdog:** Automatically halts unattended TX after a configurable timeout (default 10 minutes).
- **Hunter Auto-Answer (`--auto-answer off|new-grid|new-band|any`):** Automatically answers CQs matching your criteria.
- **Hourly UTC Rendezvous (`--rendezvous`):** Follows the coordinated hourly band/time rendezvous schedule (with 50/50 Call & Listen) so low-power stations find each other on quiet bands.
- **Logs:** Writes standard WSJT-X-style `ALL.TXT` and **ADIF (`.adi`)** QSO logs.

---

## 5. `rad950.org` "On The Air" Features & Web Remote

Operators can sign in at **`https://rad950.org`** using passwordless email magic link / 6-digit code (callsign + grid):

- **Live Spots (`/on-the-air/spots`):** Real-time reception reports and propagation map showing who hears whom on 17m/15m/12m/10m (including `★ COOP` cooperative-RX rescues). Generate a personal `r950_...` **API Key** here and paste it into the desktop app's Settings to enable automatic spotting, cloud QSO upload, and Web Remote.
- **Stats & Map (`/on-the-air/stats`):** Global QSO network map, band activity breakdown, and worked-grid visualizations.
- **Leaderboards (`/on-the-air/leaderboards`):** Top stations by QSOs, unique grids, and DX distance.
- **Upload Log (`/on-the-air/upload`):** Upload ADIF logs from the desktop app **or pull QSOs directly from an RT-950 Pro over Bluetooth / USB**.
- **Callsign Lookup (`/on-the-air/callsign/<CALL>`):** Per-station activity, spots, and confirmed QSO history.
- **Get On The Air (`/on-the-air/get-on-the-air`):** Guided setup tabs for **Handheld (RT-950 Pro)** and **Desktop** plus the browser-based **Ferrite Web Console** flasher.
- **Web Remote Control (`/on-the-air/remote`):**
  - Control any of your own `rad950` desktop, HackRF, TCI, or USB-bridged **RT-950 Pro** stations from any desktop or mobile web browser via Cloudflare Durable Object WebSocket relay.
  - Streams live 10 Hz panafall spectrum/waterfall, Band Activity, My Activity, band switching, subchannel selection, Monitor / Run Even / Run Odd, Pounce (click a heard station to answer), Hunter, Rendezvous, RAD950-60, Dry-TX / Live-TX toggle, and instant **HALT TX**.
  - **Enable on the station:** Paste your `r950_...` API key in Settings, enable **Web Remote** (`web_remote_enabled = true`), set a station name, and optionally allow TX (`web_remote_allow_tx = true`). For safety, if your browser disconnects while TX is active, the station automatically halts TX.

---

## 6. Quick Troubleshooting Checklist

| Symptom | Likely Cause | Fix |
|---|---|---|
| **Signals visible on waterfall, 0 decodes** | PC or radio clock off by > 0.5 s | Sync PC with NTP (`w32tm /resync`, Meinberg/Dimension 4) or lock RT-950 Pro clock via GPS, OTA CLK, WWV CLK, or USB Link. |
| **RT-950 Pro decodes high subchannels (4–7) but misses 0–3** | Uncalibrated SI4732 BFO offset (typically `-500` to `-800 Hz`) | Open RAD950 Setup (`*`) and raise **`RX corr`** until a 1500 Hz signal centers at 1500 Hz. |
| **RT-950 Pro says `Need UTC to TX`** | Clock not yet acquired | Acquire GPS fix outdoors, run `WWV CLK`, wait for one decoded OTA burst (`OTA CLK`), or connect USB to `rad950 --backend rt950` / Web Console. |
| **Desktop app says `DRY-TX` / rig doesn't key** | Safety Dry-TX enabled, or `rigctld` not connected | Turn off Dry-TX (`d` or GUI toggle); verify `rigctld` is running at `127.0.0.1:4532` and rig is in USB-DATA mode. |
| **RT-950 Pro bridge (`--backend rt950`) fails to connect** | Serial port busy or wrong baud/port | Close other serial apps/Web Console; pass `--rt950-port COMx` (Windows) or `/dev/serial/by-id/...` (Linux); ensure Ferrite `0.9.3` build `6017913`+ is flashed. |
| **No audio input/output on desktop** | Wrong sound card or OS mic permission denied | Run `rad950 --list-audio` (or open GUI Settings → Refresh), pick the rig's USB Audio CODEC, and allow microphone access in macOS/Windows Privacy settings. |
| **RT-950 Pro stops receiving after first TX** | Older Ferrite firmware before build `6017913` (`SI_IN` LNA restore bug) | Update to `ferrite-tx.ffw` build `6017913`+ (`v0.1.2` or newer) from `https://rad950.org`. |
