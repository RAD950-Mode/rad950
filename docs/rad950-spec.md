# RAD950 Protocol & Operating Specification (v1)

Normative specification of the **RAD950** (`urad950` / `RAD950`) HF weak-signal protocol, Run / Hunter operating modes, Hourly UTC Rendezvous, and Spots API as implemented and over-the-air verified across two radio paths (sharing the `urad950` codec crate):
- **Embedded handheld firmware:** Ferrite on Radtel RT-950 Pro (`digimodes/urad950`, `crates/ferrite-core/src/rad950.rs`, `crates/ferrite-app/src/rad950.rs`, `crates/ferrite-app/src/rad950_rdv.rs`, `firmware/src/tasks/ui.rs`).
- **Host SDR transceiver / skimmer:** `digimodes/urad950/examples/rad950_hackrf.rs` (HackRF One, 2.4 MS/s USB DDC).
- **Web Spots API:** `rad950-web/src/spots.ts`.

---

## 1. Band Plan & USB Dial Frequencies

All transmissions are **Upper Sideband (USB)** (or equivalent direct-PLL 4-FSK carrier placement at `dial_hz + f_audio`).

| Band | Primary Calling / Rendezvous Dial (USB) | Secondary Dials (USB) |
|---|---|---|
| **10 m** | **28.078 MHz** | 28.082, 28.120, 28.180 MHz |
| **12 m** | **24.919 MHz** | 24.923 MHz |
| **15 m** | **21.078 MHz** | 21.082 MHz |
| **17 m** | **18.104 MHz** | 18.108 MHz |

---

## 2. Physical Layer (PHY) & Slot Timing

### 2.1 UTC Slot Grid & Clock Discipline
- **Slot duration:** `10.000 s` aligned to UTC (`utc_sec % 10 == 0`).
  - **Even slots:** `(utc_sec / 10) % 2 == 0` (`:00`, `:20`, `:40`).
  - **Odd slots:** `(utc_sec / 10) % 2 == 1` (`:10`, `:30`, `:50`).
- **TX burst timing:**
  - Nominal start: **`+400 ms`** (`start_sym = 5`) into the 10 s slot.
  - Duration: **`8.80 s`** (`110` symbols × `80 ms`), ending at `+9.20 s`.
  - RX search window: `start_sym ∈ [1, 9]` (`+80 ms` to `+720 ms`, i.e. **`±320 ms`** around nominal).
  - RX slot decode flush: **`+9.55 s`** (`finish_slot()`). After any TX slot, the DSP receiver MUST be reset at `0.00 s` of the following RX slot so muted TX audio cannot lock out RX acquisition.
- **TX clock gate (`MAX_TX_UNCERT_MS = 150 ms`) & holdover budget:**
  - Scheduled TX is permitted only when `utc_sec != 0` and the implementation's estimated timing uncertainty `uncert_ms <= 150 ms`.
  - **Live GPS (`GPS PNT` / `GPS CLK`):** Disciplined to the NMEA `$` burst edge, budgeted at `50 ms` base timestamp uncertainty.
  - **Crystal holdover (`GPS HOLD`):** After a valid onboard GPS fix, the RT-950 Pro firmware budgets `uncert_ms = 50 + elapsed_secs / 20` (`50 ms` starting error + `100 ms` drift allowance at `50 ppm` = `2,000 s` / `33.3 min` before crossing `150 ms`). Actual holdover accuracy depends on the initial NMEA timestamp error and oscillator drift staying within those bounds; other radios should parameterize the same `150 ms` TX gate for their own clock source.
  - **Bench fallback (`BENCH CLK`):** Compile-time `bench` builds only; release builds require GPS lock or active holdover (`NO GPS` otherwise).

### 2.2 Modulation & Passband Layout
- **Modulation:** Phase-continuous **4-FSK**, tone spacing **`50 Hz`**, symbol period **`80 ms`** (`10 ms` raised-cosine ramp + `70 ms` steady tone).
- **Gray dibit-to-tone mapping:** `(b0, b1)` → tone `0..3`: `00 → 0`, `01 → 1`, `11 → 2`, `10 → 3` (`GRAY_ENCODE = [0, 1, 3, 2]`).
- **8 TX subchannels (`SUBCHANNELS_HZ`, lowest tone `f0` above USB dial):**
  - `600, 825, 1050, 1275, 1500, 1725, 1950, 2175 Hz` (`225 Hz` step = `150 Hz` tone span / `~200 Hz` occupied bandwidth + `25 Hz` guard; shared across core PHY, Run mode, and Rendezvous).
- **Continuous RX search:** `585.9–2,332.0 Hz` (`150` FFT bins at `11.72 Hz`/bin, `6,000 Hz` decimated processing rate), up to **`8` simultaneous tracks/decodes** (`MAX_TRACKS = MAX_DECODED = 8`) per slot, with `±11.72 Hz` linear frequency-drift tracking across the burst tail.

### 2.3 Sync & Interleaving
- **110 symbols total:** `18` sync symbols + `92` data symbols (`184` coded bits).
- **Sync placement (`6 + 6 + 6`):**
  - `S0` at symbols `0..6`: `[0, 3, 0, 1, 3, 2]`
  - `S1` at symbols `18..24`: `[3, 0, 3, 2, 0, 1]`
  - `S2` at symbols `104..110`: `[3, 1, 0, 2, 3, 0]`
- **Interleaver:** Stride-`37` bit permutation across the `184` coded bits (`gcd(37, 184) = 1`), spreading adjacent codeword bits `~3 s` apart against fading:
  - Direction: `inter[(37 * i) % 184] = codeword[i]` for `i in 0..184`.
  - Data symbol `d in 0..92` maps to frame symbol `6 + d` (for `d < 12`) or `24 + (d - 12)` (for `d >= 12`), carrying dibit `(inter[2*d] << 1) | inter[2*d + 1]` through `GRAY_ENCODE`.

---

## 3. FEC & 64-Bit Wire Frame Format

### 3.1 CRC-14 & LDPC(184, 78)
- **CRC-14:** Computed MSB-first over the 64-bit payload:
  - Polynomial: `0x2757` (`x^14 + x^13 + x^10 + x^8 + x^6 + x^4 + x^2 + x + 1`, FT8/FT4 family).
  - Initial register: **`0x3FFF`** (prevents all-zero codewords from validating as frames).
  - Final XOR: **`0x0000`** (none).
  - Systematic message `msg[0..78]`: `msg[0..64]` = payload MSB-first, `msg[64..78]` = CRC-14 MSB-first.
- **LDPC `(184, 78)`:** Systematic PEG LDPC code (`K = 78` message bits, `M = 106` parity checks, `N = 184` coded bits, girth 6, zero 4-cycles, bridged dual-diagonal parity):
  - Parity column `78` (`p_0`) connects to check rows `0`, `53`, and `105` (degree 3); parity columns `78 + c` (`c = 1..105`) connect to check rows `c - 1` and `c` (degree 2).
  - Linear-time systematic encoding: compute message syndrome `s[r] = XOR_{j: r in MSG_ROWS[j]} msg[j]` for `r in 0..106`; then `p_0 = XOR_{r=0..106} s[r]`, `p_1 = s[0] ^ p_0`, and `p_c = p_{c-1} ^ s[c-1] ^ (p_0 if c - 1 == 53 else 0)` for `c = 2..105`.
  - Reference decoder: integer normalized min-sum BP (`scale = 3/4`, up to `35` iterations).
  - **Normative `MSG_ROWS[0..78]`** (the 4 check rows per systematic message bit `0..77`):
    ```text
    0..12:  [17,44,79,97] [6,31,66,89]  [28,48,59,73] [11,38,56,84] [24,40,91,102] [4,14,22,50]
            [61,69,81,92] [2,36,64,77]  [8,41,52,75]  [13,33,70,99] [27,87,96,105] [3,10,46,93]  [19,74,83,100]
    13..25: [34,42,58,94] [16,29,39,68] [21,32,62,86] [23,54,65,72] [49,67,78,103] [7,12,63,104]
            [47,55,90,98] [5,37,95,101] [15,35,82,88] [1,25,30,57]  [43,51,71,85]  [20,57,76,80] [0,9,18,60]
    26..38: [26,45,49,70] [20,46,53,102][62,67,83,94] [0,33,39,101] [4,27,34,55]   [13,28,66,76]
            [12,17,54,92] [30,37,47,51] [11,15,32,52] [7,24,35,48]  [64,68,71,90]  [2,6,60,85]   [40,65,79,86]
    39..51: [10,23,29,99] [21,41,81,96] [3,58,91,97]  [16,25,42,61] [8,26,84,98]   [1,72,88,103]
            [5,44,77,105] [19,22,38,59] [9,31,45,63]  [18,56,75,93] [43,74,80,104] [50,69,73,95] [14,78,87,100]
    52..64: [36,53,89,97] [25,38,44,82] [12,30,73,86] [9,27,68,82]  [59,63,88,98]  [2,47,70,75]
            [4,29,36,62]  [33,43,65,96] [14,26,58,72] [17,52,67,101][32,48,92,105] [7,18,42,50]  [16,37,66,104]
    65..77: [21,60,78,90] [31,41,54,100][11,34,45,80] [10,69,79,83] [1,13,55,81]   [8,28,71,102]
            [5,23,61,74]  [39,51,77,94] [24,76,84,87] [40,56,89,99] [15,49,57,64]  [3,19,95,103] [0,22,66,91]
    ```

### 3.2 Field Encodings
- **28-bit amateur callsign (`0 ..= 262,177,559`):** Standard 6-slot `[C0, C1, C2, C3, C4, C5]` with call-area digit at `C2`:
  - `C0`: `' '`, `'A'..'Z'`, `'0'..'9'` (37) · `C1`: `'A'..'Z'`, `'0'..'9'` (36) · `C2`: `'0'..'9'` (10) · `C3..C5`: `' '`, `'A'..'Z'` (27 each).
- **15-bit Maidenhead `grid4`:** `0..=32,399` (`AA00..RR99`); `32,400` (`GRID_UNKNOWN`) = unset/no grid.
- **5-bit `snr_db`:** `[-24, +7] dB` in `1 dB` steps (`code = clamp(snr_db, -24, 7) + 24`, 2,500 Hz reference bandwidth).
- **4-bit `tx_pwr`:** `0` = unspecified; `1..=15` = `tx_pwr_dbm = (code - 1) * 3` dBm:
  - `1` = `0 dBm` (1 mW), `2` = `+3 dBm` (2 mW), `3` = `+6 dBm` (4 mW), `4` = `+9 dBm` (8 mW), `5` = `+12 dBm` (16 mW), `6` = `+15 dBm` (32 mW), `7` = `+18 dBm` (63 mW), `8` = `+21 dBm` (125 mW), `9` = `+24 dBm` (250 mW), `10` = `+27 dBm` (500 mW), `11` = `+30 dBm` (1 W), `12` = `+33 dBm` (2 W), `13` = `+36 dBm` (4 W), `14` = `+39 dBm` (8 W), `15` = `+42 dBm` (~16 W).

### 3.3 Frame Layouts (MSB = bit 63)

| Frame | Bit Layout (64 bits, MSB → LSB) | Notes |
|---|---|---|
| **`Frame::Cq`** (Type 0) | `1b type=0` \| `28b sender` \| `15b grid4` \| `4b tx_pwr` \| `16b reserved` | **`reserved & 0x0F`** encodes optional numeric callsign suffix (`0` = none, `1..=15` = `-1`..`-15`, e.g. `NQ6N-1`). |
| **`Frame::Rx`** (Type 1) | `1b type=1` \| `28b sender` \| `28b recipient` \| `5b snr` \| `1b ack` \| `1b reserved` | `ack=0` is initial report (`RX`); `ack=1` is confirmed report (`R+RX`). Carries base 28-bit callsigns only (no suffix). |
| **`Frame::ExtFrag`** | `1b type=1` \| `6b marker=0b111111` \| `2b kind` \| `2b idx` \| `8b tag` \| `45b payload` | Conversation / Distress extension fragments; rejected as invalid callsign (`>= 0x0FC0_0000`) by core `Rx` unpackers. |

---

## 4. QSO State Machine & Operating Modes

### 4.1 Core QSO Invariants
1. **Strict alternating TX/RX turns:** After any TX slot `s`, the station MUST listen during slot `s + 1` (`next_tx >= s + 2`).
2. **Opposite-parity reply rule:** When answering a station last heard in slot `s_heard`, the reply MUST be transmitted on parity `(s_heard + 1) % 2` so replies never collide with an Auto-CQ caller's TX slots.
3. **CQ-only suffix scope & self-base-call suppression:**
   - Numeric suffixes (`-1..-15` in `Frame::Cq.reserved & 0x0F`) identify individual rigs/antennas in `CQ` decodes and `POST /api/spots`. CQs sharing a base callsign with a *different* suffix (`sender == my_call && suffix != my_suffix`) are decoded and shown in Heard.
   - `Frame::Rx` carries base 28-bit callsigns only (no suffix). Consequently, if two rigs sharing a base callsign (e.g., `NQ6N-1` and `NQ6N-7`) operate on the same dial, a hunter can distinguish their CQs, but a directed reply addresses `NQ6N` and **both rigs recognize it as addressed to them**.
   - Directed QSOs and auto-answers strictly require `sender != my_call` (a station never works its own base callsign).
4. **3-slot QSO completion, causal `ACK=1`, and asymmetry tradeoff:**
   - **Caller (CQer):** `Slot N`: TX `CQ` → `Slot N+1`: RX `RX(ACK=0)` from responder → `Slot N+2`: TX `R+RX(ACK=1)` → **log QSO upon confirmed clean TX completion** (`confirm_tx_completed`).
   - **Responder:** `Slot N`: RX `CQ` → `Slot N+1`: TX `RX(ACK=0)` → `Slot N+2`: RX `R+RX(ACK=1)` → **log QSO only if `ACK=1` causally follows a completed `RX(ACK=0)` sent to that peer within `EXPIRY_SLOTS = 30` (5 min)**. Cold/unsolicited `ACK=1` frames are ignored.
   - **3-slot tradeoff:** Skipping a 4th `73` slot keeps clean QSOs to 30 s, and the 2-slot linger gives a faded `ACK=1` another chance if the responder retries `RX(ACK=0)`. However, because the caller logs after clean TX of `ACK=1`, a one-sided log is still possible if `ACK=1` never decodes at the responder before the linger expires.
5. **Post-Completion Linger, Pileup Retention & Cooldown:**
   - After sending `ACK=1`, linger on the window for `LINGER_SLOTS = 2` slots (20 s) so a faded `ACK=1` can be retried (up to `MAX_COOL_RETRIES = 2` within `COOLDOWN_SLOTS = 30`, never writing a duplicate log entry).
   - Unanswered TX retries stop after `MAX_TRIES = 4`, **but the peer session (`tx_r0_done_at`) MUST remain alive until `EXPIRY_SLOTS = 30` (5 min)** so a Runner working through a multi-station pileup can still deliver a valid `R+RX(ACK=1)` after the caller stops retrying without causing a one-sided log.
6. **Hardware relay guard (`SAFE-22`):** On handheld hardware where TX/RX switches an RF relay, at most `MAX_TX_PER_MIN = 3` TX slots per rolling `6`-slot (`60 s`) window are permitted.
7. **Busy-subchannel avoidance:** Any subchannel decoded during an RX slot is marked occupied; `pick_subchannel()` chooses an unoccupied subchannel out of the 8 for the next TX.

### 4.2 Operating Modes
- **Monitor (`OpMode::Monitor`):** RX-only; decodes all frames into the Heard list with live award classification (`★ NEW` = unworked grid all-time, `+ BND` = unworked grid on current band, `✓ WKD` = worked).
- **Run / Auto-CQ (`OpMode::AutoCq { parity }`):** Calls `CQ` on slots matching `parity` (even or odd) whenever no directed peer reply is pending; automatically replies `RX(ACK=0)` to incoming CQs and `R+RX(ACK=1)` to incoming `RX(ACK=0)` answers.
- **Manual Answer (`OpMode::Pounce`):** Operator selects a heard station and triggers `Answer`; queues opposite-parity `RX(ACK=0)` (or `R+RX` if the peer already sent `RX(ACK=0)`).
- **Attended Hunter / Auto-Answer (`AutoAnswer`):**
  - Modes: `Off` → `NewGrid` (`HUNT:NEW`) → `NewBandGrid` (`HUNT:BND`) → `AnyNeeded` (`HUNT:ALL`).
  - Priority when multiple CQs decode in the same slot: **`NewGrid` > `NewBandGrid` > `NewCall` > highest SNR**.
  - When armed and idle in `Monitor`, automatically answers matching CQs (and replies `R+RX` if called directly), logs the completed QSO, and immediately releases back to `Monitor` to hunt the next needed station.
  - Never duplicates an already-worked callsign on the same band (or an already-worked grid when restricted to `NewGrid` / `NewBandGrid`).

---

## 5. Hourly UTC Rendezvous Mode

Published 5-minute (`30`-slot) meetup windows every hour so low-power portable stations find each other without internet spotting:

| UTC Minutes | Band | USB Calling Dial |
|---|---|---|
| **`:00` – `:04`** | **10 m** | `28.078 MHz` |
| **`:15` – `:19`** | **12 m** | `24.919 MHz` |
| **`:30` – `:34`** | **15 m** | `21.078 MHz` |
| **`:45` – `:49`** | **17 m** | `18.104 MHz` |

### 5.1 Rendezvous Rules
1. **Schedule following (`follow_schedule`):** Automatically retunes to the active enabled band when its 5-minute window opens. Retuning is **deferred while a directed QSO is in progress** (`has_active_qso == true`).
2. **Independent 50/50 Call & Listen (`call_listen_armed`):**
   - Only **even slots** (`slot % 2 == 0`) are eligible for Rendezvous CQs; **odd slots** are always `Listen` (reserved for initial `RX(ACK=0)` replies; the caller's final `R+RX(ACK=1)` lands on the following even slot). Multiple hunters may still answer the same CQ in the same odd slot (typically separated across the 8 subchannels, or colliding if they pick the same subchannel).
   - Seed: `seed = (call.raw() ^ ((suffix as u32) << 24)).wrapping_mul(0x9E37_79B1) ^ 0x5A3C_9E37` (or `0x1357_9BDF` if 0).
   - Turn draw (`busy_cnt = occupied_subch_mask.count_ones()` from previous RX slot):
     - `r = mix32(seed ^ slot.wrapping_mul(0x9E37_79B1))`
     - Adaptive congestion backoff: transmit CQ iff `(r & mask_p) == 1`, where `mask_p = 1` (`50%`) for `busy_cnt < 4`, `mask_p = 3` (`25%`) for `4 <= busy_cnt <= 5`, and `mask_p = 7` (`12.5%`) for `busy_cnt >= 6`.
   - Subchannel pick: `mix32(r ^ 0x85EB_CA6B) % clear_count` over unoccupied subchannels (`!occupied_subch_mask`).
   - `mix32(x)`: `x ^= x >> 16; x *= 0x7FEB_352D; x ^= x >> 15; x *= 0x846C_A68B; x ^ (x >> 16)`.
   - `call_listen_armed` automatically clears when the 5-minute window closes.
3. **Answering Rendezvous CQs:** When a Rendezvous station in `Monitor` receives an incoming `RX(ACK=0)` addressed to `my_call`, it promotes out of `Monitor` to transmit `R+RX(ACK=1)`, logs the QSO, and returns to `Monitor` so it can answer subsequent callers or resume Rendezvous turns.

---

## 6. Live Spotting API (`POST /api/spots`)

Skimmers and host SDRs (`rad950_hackrf --spot`) upload decoded slots asynchronously to `POST /api/spots` (default `https://rad950.com/api/spots`):

- **Authentication:** `Authorization: Bearer <r950_...>` (personal skimmer key from `POST /api/spots/key`, SHA-256 hashed server-side) or signed-in browser session.
- **Request body:** JSON `{"spots": [ ... ]}` (up to `100` spots per batch):
  ```json
  {
    "spots": [
      {
        "spotter_call": "NQ6N",
        "spotter_grid": "EN61",
        "tx_call": "W2BBB",
        "tx_grid": "FN31",
        "utc_sec": 1795154400,
        "freq_hz": 28079050,
        "band": "10m",
        "snr_db": -6,
        "tx_pwr_code": 10,
        "frame_type": "CQ"
      }
    ]
  }
  ```
- **Rules:**
  - `freq_hz = dial_hz + audio_hz`; `frame_type` is `"CQ"`, `"RX"`, or `"R+RX"`.
  - `spotter_grid`, `tx_grid`, and `tx_pwr_code` are omitted when unknown/`0`.
  - Self-base-call spots (`base(tx_call) == base(spotter_call)`) are skipped client-side and ignored server-side.

---

## 7. Golden Interoperability Test Vectors

Verified against `urad950::tests::golden_spec_test_vectors`:

### Vector 1: `CQ NQ6N EN61` (`tx_pwr = 11` / `+30 dBm`, `suffix = 0`)
- **`payload64`:** `0x30DDC102171B0000`
- **`crc14`:** `0x28CC`
- **`ldpc184` (46 hex nibbles, MSB-first):** `0x30DDC102171B0000A330AACDF5B13048236A473BC10846`
- **`tones110` (`0..3`, 110 symbols):**
  `03013201020002310130320102223001300302032210012013201100032100130122210200002200030300100130332133331200310230`

### Vector 2: `R+RX K1ABC de NQ6N -12 dB` (`ack = 1`)
- **`payload64`:** `0xB0DDC1000F2C50B2`
- **`crc14`:** `0x2574`
- **`ldpc184` (46 hex nibbles, MSB-first):** `0xB0DDC1000F2C50B295D059FB32E01537E56358A0C06E80`
- **`tones110` (`0..3`, 110 symbols):**
  `03013231001001021130320112103301303131110303001033201302313312123031210303113301223010100121202231100313310230`

---

## 8. `RAD950-60` 1-Minute Weak-Signal Cycle (v1-Compatible)

Optional 60-second UTC minute cycle (`utc_sec / 60`, alternating even/odd minutes) using the **exact same v1 wire format**:
- **Sub-slots `0..=4` (`:00, :10, :20, :30, :40`):** TX repeats the identical `8.80 s` v1 burst 5 times; RX stacks all 5 sub-slots into a folded passband spectrogram (`urad950::Receiver60`, `116 × 150` 5-bit cells = `10,875 B`, fitting inside Ferrite's shared `13 KB` `Decoder` union slot).
- **Sub-slot `5` (`:50–:59`):** 10-second PA cooldown and decode turnaround guard (no TX; RX runs blind `S0+S1+S2` stacked-sync acquisition and `(184, 78)` LDPC decode).
- **Compatibility & crossover:** Standard 10 s v1 receivers decode strong `RAD950-60` transmissions on the first 10 s burst; below `~−18.5 dB` (2,500 Hz ref) where 10 s v1 drops to 0 QSOs/hr, `RAD950-60` extends blind decode down to `−19..−20 dB` (`4–18` QSOs/hr).


