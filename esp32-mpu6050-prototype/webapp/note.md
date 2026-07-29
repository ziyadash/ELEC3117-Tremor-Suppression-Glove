# tremor detection notes

Two independent detector chains (gyro, accel) run in parallel; tremor =
either one fires. Accel is DMP-compensated linear accel (gravity already
removed), not raw accelerometer data.

- sample rate: fixed 50Hz, matching the firmware's paced BLE notify rate
- per channel, per axis: 3-12Hz Butterworth bandpass -> 0.6s sliding RMS,
  combined across axes as sqrt(rmsX²+rmsY²+rmsZ²)
- calibration: ~2s resting baseline on enable; threshold = 85th
  percentile of that baseline's RMS * 1.6, clamped to a floor/ceiling
  (gyro 1.5-15 deg/s, accel 0.03-0.12g)
- gate 1, energy: combined RMS > threshold
- gate 2, oscillatory: |mean|/rms < 0.45 on the dominant axis (rules out
  ramps/jerks - a zero-mean oscillation nets ~0, a ramp nets ~its own RMS)
- gate 3, sustained: a 0.15s recent-RMS window stays >=55% of the full
  window's RMS (rules out a filter's ringdown after a one-off jerk, which
  passes gates 1-2 but decays fast)
- persistence: gyroPasses||accelPasses must hold 300ms -> ACTIVE, clear
  for 150ms -> idle
- freq: zero-crossing count on the leading channel's dominant axis,
  display only

# 3D asset attribution

The hand model rendered in the web app viewport (webapp/js/scene.js,
webapp/assets/models/RightHand.obj) is "Low Poly Right Hand" by Raziq
Brown, sourced from Poly Pizza (https://poly.pizza/m/cl8ax5B13cp),
licensed under Creative Commons Attribution 3.0
(https://creativecommons.org/licenses/by/3.0/). No changes to geometry
beyond recentering/rescaling for display; material/color is applied by
the app, not part of the original asset.
