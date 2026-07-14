# tremor detection notes

- input: accel (g), already through display lowpass (alpha=0.3), not raw
- per axis: bandpass 3-8Hz (butterworth hp+lp biquad, fixed @ 50Hz)
- square it, one-pole envelope (attack/release ~30ms, fast both ways)
- combine: rms = sqrt(envX+envY+envZ) -> rotation invariant
- threshold 0.03g, on-hold 40ms, off-hold 0ms -> ACTIVE/idle
- freq: zero-crossing on dominant axis, not fft (too slow @ 50Hz), display only
- no gyro, no fft, single fixed threshold - simple & fast, not fancy
