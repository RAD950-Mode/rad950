# RAD950 receiver performance review

Reviewed against commit `1ccb072`. Both receive modes use a 256-point complex FFT to analyze 512 real samples. The butterfly at index k uses the same complex rotation in every block of a stage. Computing that rotation once per k reduces sine/cosine lookups from 1,024 to 255 per transform, with the existing Q15 arithmetic, exponent decisions, and storage. The traversal trades contiguous accesses within a block for reuse across blocks.

## Host result

On arm64 macOS with rustc 1.92.0 and `rustc -O`, median transform time fell from 3.11 µs to 2.57 µs (17% less elapsed time). The temporary standalone benchmark compared the original and reordered functions in the same executable. It used 128 deterministic 512-element i16 inputs at ten amplitude scales, 20,000 transforms per timed batch, six paired batches with alternating order, and a black box around inputs and results. All 128 output buffers and returned exponents matched exactly. The timing includes the harness's array copies. End-to-end decoder latency and Cortex-M4F cycle counts remain unmeasured. On the handheld, compare cycles per transform and worst-case audio-block processing time before adopting the change.

A second host experiment tracked the next stage's maximum while writing butterfly outputs. It matched the output and shift count on 10,000 random buffers. Over 2,000 transforms this took 7.55 ms versus 5.36 ms with the separate scans, so the implementation retains the scans.

## Minute-receiver sensitivity experiment

Run `cargo run --release -p urad950 --example rx60_sweep -- 32 101` for the validation set (trial count and first seed). Each CSV row reports weak, strong, and unexpected frame counts. At each seed the cases reuse the same five independent pseudo-Gaussian noise segments, with a target at +400 ms and an optional second CQ at +440 ms, 225 Hz below and 18 dB stronger. Nominal input SNR uses the 2,500 Hz reference. The experiment compares exact decoded frames. A noise-only minute per seed checks for false accepts.

**Acquisition fix.** On seeds 1–16 the original search missed the 2175 Hz target in every condition. It stopped at half-bin 269, while the nominal carrier falls near half-bin 271. Deriving the bound from tone 3 and its required interpolation neighbor admits indices through 272. At −20 dB the fix recovered 16/16 in isolation and 12/16 with interference. On the same seeds the 1275 Hz weak-frame counts stayed identical across all ten conditions.

**Quantizer comparison.** On the original search, a candidate used the upper end of the approximate 95th percentile (eight-code histogram buckets) to set the six-level span. At 1275 Hz and −21 dB with interference it recovered 9/16 weak frames versus 7/16 for the peak-based span, but strong-frame recovery fell from 10/16 to 8/16. Across the five central adjacent-signal conditions strong-frame recoveries fell from 49/80 to 35/80. Retain the peak-based span. The draws are paired across conditions, so these summed counts describe the experiment and do not represent 80 independent seeds.

**Validation on fresh seeds 101–132.** Counts below are target decodes out of 32 with the corrected search and original quantizer.

| Target and condition | −23 dB | −22 dB | −21 dB | −20 dB | −19 dB |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1275 Hz alone | 0 | 5 | 23 | 32 | 32 |
| 1275 Hz with neighbor | 0 | 2 | 12 | 32 | 32 |
| 2175 Hz alone | 0 | 6 | 24 | 29 | 32 |
| 2175 Hz with neighbor | 0 | 1 | 9 | 19 | 27 |

There were zero unexpected frames, including in 32 noise-only minutes. Strong-neighbor recovery at the upper channel was only 12/32 at the −19 dB target setting. The neighbor begins half a symbol later than the target, while the minute receiver evaluates one FFT timing per symbol. A paired timing-offset sweep could isolate that effect. These trials cover fixed timing, two payloads, and simulated noise. Hardware performance and fading channels require separate measurements.

## Complementary desktop minute decode

The desktop minute receiver runs both quantizers on the same capture. It keeps the peak-based result for duplicate frames and accepts additional decoded frames from the percentile pass. Both use the same acquisition, LDPC, and CRC checks. The alternate span takes the upper edge of the eight-code histogram bucket containing the eighth-highest of 150 log-power bins. The ordinary Receiver60 constructor selects peak-based folding.

### Paired experiment

For each trial, both receivers saw identical PCM, carrier placement, payloads and timing, with the corrected frequency search. Results below sum five nominal target SNRs from −23 through −19 dB. Each row includes a neighbor 18 dB stronger and 225 Hz lower in frequency. Counts are frame opportunities across the sweep. SNR points within a seed share noise.

| Target | Neighbor delay relative to target | Opportunities | Weak frames, peak → union | Strong frames, peak → union |
| --- | ---: | ---: | ---: | ---: |
| 1275 Hz | +40 ms | 320 | 164 → 177 | 158 → 230 |
| 2175 Hz | +40 ms | 320 | 114 → 159 | 145 → 186 |
| 1275 Hz | 0 ms | 160 | 83 → 91 | 160 → 160 |
| 2175 Hz | 0 ms | 160 | 38 → 71 | 160 → 160 |

The second pass also recovered 11 additional isolated targets across the two seed sets. At seed 1010, 2175 Hz, −20 dB and +40 ms, the peak decoder returned only the neighbor and the percentile decoder returned only the target. Counting unique payloads retained both. The union yielded zero unexpected frames in the 1,920 signal-bearing captures and 352 independent noise-only minutes. The noise-only count gives a one-sided 95% upper bound of about 0.85% per simulated minute with any false accept under this noise model.

The first 1,344 release-mode cases on arm64 macOS took a median 8.02 ms single-pass and 16.12 ms dual-pass, with an observed dual-pass maximum of 19.73 ms. These timings include synthesised PCM mixing, receiver initialization, ingestion and final decoding. They exclude device I/O, desktop resampling, UI dispatch and logging. The desktop captures 50 seconds of a 60-second turn. This comparison measures decoder cost within that nominal 10-second turnaround. A separate release-mode desktop smoke test took 18.7 ms from a complete 12 kHz memory capture through normalization, both decoders and the merge, producing one copy of the expected CQ. CPU load and backend timing on other machines may differ.

Reproduce from the workspace root. CSV output includes per-pass recovery flags and times in microseconds. The third argument sets the neighbor start in milliseconds from the subslot boundary. The target starts at +400 ms.

```sh
cargo run --release -p urad950 --example rx60_sweep -- 64 1001 440
cargo run --release -p urad950 --example rx60_sweep -- 32 2001 400
cargo run --release -p urad950 --example rx60_sweep -- 256 9001 400 noise-only
```

The results support an automatic desktop second pass. They apply to synthetic stationary signals with fixed payloads. Fading, clock drift, crowded real-band captures and handheld CPU cost remain outside this experiment.
