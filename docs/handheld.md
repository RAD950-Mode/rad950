# Embedding the RAD950-60 receiver

Add the `urad950` crate from this repository to your firmware. It uses the workspace MIT license and contains its own fixed-point DSP. For a local checkout next to your firmware project, use this dependency in your firmware's Cargo.toml.

```toml
[dependencies]
urad950 = { path = "../rad950/crates/urad950" }
```

Use `urad950::CompactReceiver60` for the compact two-pass receiver. It processes each symbol's FFT once. A small table retains extra tone information at the eight published channel frequencies, so a second decode can recover frames masked by a strong neighbor. The primary pass still searches between the published frequencies. The combined result contains each payload once and gives primary decodes priority when the output buffer fills.

## Audio and timing

| UTC within the minute | Firmware action |
| --- | --- |
| :00 | Call `reset()` before feeding the first audio block. |
| :00 through :49 | Feed contiguous signed 16-bit, mono PCM at 24,000 Hz to `push_samples()`. At :10, :20, :30 and :40, call `start_subslot()` before feeding the first sample of the new subslot. Split blocks that straddle a boundary. |
| :50 | Once the fifth subslot has been consumed, call `finish_minute(&mut out)` with a `[Decoded; MAX_DECODED]` buffer. If the call returns `n`, process `&out[..n]`. |
| Next :00 | Reset the receiver for the next minute. |

The host owns the sample clock, audio capture, UTC alignment, radio tuning and result delivery. A hardware audio interrupt can enqueue blocks for a decoder task. Run the minute-end LDPC search in that task and measure its worst-case latency against the 10-second turnaround. The decoder reuses its state across minutes. Allocate space for the persistent receiver, the output buffer, and the decoder task's call stack separately.

The [protocol specification](rad950-spec.md) gives the tone frequencies, burst timing and frame layout.

## Memory and verification

`CompactReceiver60::new()` is a `const fn`, so firmware can initialize the receiver in static storage and give a single task exclusive access. The receiver object occupies **13,164 bytes** in a release build for `thumbv7em-none-eabihf`. This figure excludes stack space used during FFT and LDPC processing. The second pass can try up to 16 additional LDPC decodes per minute.

To replay synthetic 24 kHz audio against the receiver on a host, run the following from the repository root.

```sh
cargo run --release -p urad950 --example handheld_replay -- 48 12001
```

With seeds 12001–12048, a −21 dB target (2,500 Hz reference bandwidth), and a neighbor 18 dB stronger, 225 Hz below and 40 ms later, the primary and combined passes recovered 34 and 39 targets respectively at 1275 Hz, and 13 and 32 at 2175 Hz. A separate run with seeds 40801–40848 recovered 27 and 32 at 1275 Hz, and 15 and 32 at 2175 Hz. Neither run produced an unexpected payload in the signal trials or a frame in its 48 noise-only minutes. These measurements use simulated audio.
