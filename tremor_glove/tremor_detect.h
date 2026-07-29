#pragma once
#include <stdint.h>

struct TremorResult {
  bool active;
  uint8_t lead_channel;  /* 0 = gyro, 1 = accel - whichever is further above its own threshold */
  float rms;             /* leading channel's combined bandpassed RMS (deg/s or g, see lead_channel) */
  float threshold;       /* leading channel's adaptive threshold, same units */
  float freq_hz;         /* zero-crossing frequency estimate, 0 when not active */
};

/* Resets all filter/calibration/hysteresis state. Call once at boot;
   calibration then runs automatically over the first ~2.6s of samples
   fed to tremor_detect_update() - keep the glove still during that. */
void tremor_detect_init();

/* One sample through the detector. gx/gy/gz in deg/s (raw gyro), ax/ay/az
   in g (gravity-compensated linear accel). now_ms must be a monotonic
   millisecond timestamp (e.g. millis()), and calls must come at a fixed
   50Hz - the window sizes below are sized in samples for that rate. */
TremorResult tremor_detect_update(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   uint32_t now_ms);

/* ---------------------------------------------------------------------
   Internal to tremor_detect.ino, not part of the public API above.
   Declared here (rather than inline in the .ino) because the Arduino
   build hoists auto-generated function prototypes to the top of each
   .ino file, above any type defined later in that same file - putting
   the types in the included header sidesteps that entirely.
   --------------------------------------------------------------------- */
#define TREMOR_RMS_WINDOW_SAMPLES 30 /* round(0.6s * 50Hz) */
#define TREMOR_CALIB_MAX_SAMPLES  100 /* 2000ms @ 50Hz */
#define TREMOR_MAX_CROSSING_SAMPLES 6

struct Biquad {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
};

struct TremorBandpass {
  Biquad hp, lp;
};

struct SlidingStats {
  float buf[TREMOR_RMS_WINDOW_SAMPLES];
  int capacity;
  int idx, count;
  double sum, sumSq;
};

struct TremorChannel {
  TremorBandpass bpX, bpY, bpZ;
  SlidingStats rmsX, rmsY, rmsZ;
  SlidingStats shortX, shortY, shortZ;
  bool calibrating;
  bool calibStarted;
  uint32_t calibStartMs;
  float calibSamples[TREMOR_CALIB_MAX_SAMPLES];
  int calibCount;
  float adaptiveThreshold;
};

struct ChannelResult {
  bool passesAllGates;
  float combinedRms;
  float dominantBp;
  float thresholdRatio; /* -1 while calibrating */
};
