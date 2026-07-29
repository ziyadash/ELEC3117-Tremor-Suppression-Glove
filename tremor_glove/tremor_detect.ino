/* Direct port of esp32-mpu6050-prototype/webapp/js/tremor.js to C, run
   on-device instead of client-side in the browser. Every tunable and the
   gate logic are unchanged from the JS version - see that file's header
   comment for the full rationale. Summary:

   Dual-channel: tremor can show up as rotational energy (gyro) or
   translational energy (accel), so this runs two independent, identically
   structured detector chains and calls it tremor if *either* passes.

   Pipeline, per channel, per axis (x,y,z), every sample:
     1. 3-12Hz Butterworth bandpass
     2. 0.6s sliding RMS, combined across axes into one magnitude
     3. Combined RMS checked against an adaptive threshold, calibrated
        per-channel from a resting baseline (85th percentile * margin,
        clamped to floor/ceiling)

   A channel counts as tremor once all of these hold:
     - energy: combined RMS exceeds that channel's threshold
     - oscillatory: dominant axis's mean is small relative to its RMS
     - sustained: a short recent-energy window stays close to the full
       window's RMS (rejects a bandpass filter's ringdown after a jerk)

   gyroPasses || accelPasses then needs a short dwell time before the
   shared state flips ACTIVE, and a shorter dwell before dropping to idle.

   Runs at a fixed 50Hz - tremor_glove.ino paces IMU reads to match.

   The internal types (Biquad, TremorBandpass, SlidingStats, TremorChannel,
   ChannelResult) live in tremor_detect.h, not here - see that file for why. */
#include "tremor_detect.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- Tunables (identical to tremor.js) ---------------------------------- */
#define TREMOR_BAND_LOW_HZ   3.0f
#define TREMOR_BAND_HIGH_HZ  12.0f
#define FS_HZ                50.0f
#define SHORT_WINDOW_SAMPLES 8   /* round(0.15s * 50Hz) - runtime-only, no struct depends on it */
#define SUSTAIN_RATIO_THRESHOLD 0.55f
#define CALIBRATION_MS       2000u
#define CALIB_PERCENTILE     0.85f
#define CALIB_MARGIN         1.6f
#define GYRO_ABS_FLOOR_DPS   1.5f
#define GYRO_ABS_CEILING_DPS 15.0f
#define ACCEL_ABS_FLOOR_G    0.03f
#define ACCEL_ABS_CEILING_G  0.12f
#define NET_DISPLACEMENT_RATIO_THRESHOLD 0.45f
#define PERSIST_ON_MS  300u
#define PERSIST_OFF_MS 150u
#define MIN_CROSSING_INTERVAL_S 0.02f
#define MAX_CROSSING_INTERVAL_S 1.0f

/* ---- Biquad (RBJ audio-EQ-cookbook, Butterworth Q) ----------------------- */
static float biquad_process(Biquad *bq, float x) {
  float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2
          - bq->a1 * bq->y1 - bq->a2 * bq->y2;
  bq->x2 = bq->x1; bq->x1 = x;
  bq->y2 = bq->y1; bq->y1 = y;
  return y;
}

static Biquad biquad_design(bool highpass, float fc, float fs) {
  Biquad bq = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  float w0 = 2.0f * (float)M_PI * fc / fs;
  float cosw0 = cosf(w0), sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * 0.70710678f); /* 1/sqrt(2), Butterworth Q */
  float a0 = 1.0f + alpha;
  if (!highpass) {
    bq.b0 = ((1.0f - cosw0) / 2.0f) / a0;
    bq.b1 = (1.0f - cosw0) / a0;
    bq.b2 = ((1.0f - cosw0) / 2.0f) / a0;
  } else {
    bq.b0 = ((1.0f + cosw0) / 2.0f) / a0;
    bq.b1 = (-(1.0f + cosw0)) / a0;
    bq.b2 = ((1.0f + cosw0) / 2.0f) / a0;
  }
  bq.a1 = (-2.0f * cosw0) / a0;
  bq.a2 = (1.0f - alpha) / a0;
  return bq;
}

static void bandpass_init(TremorBandpass *bp) {
  bp->hp = biquad_design(true, TREMOR_BAND_LOW_HZ, FS_HZ);
  bp->lp = biquad_design(false, TREMOR_BAND_HIGH_HZ, FS_HZ);
}

static float bandpass_process(TremorBandpass *bp, float x) {
  return biquad_process(&bp->lp, biquad_process(&bp->hp, x));
}

/* ---- Sliding-window mean/RMS (O(1) per sample) --------------------------- */
static void sliding_init(SlidingStats *s, int capacity) {
  memset(s, 0, sizeof(*s));
  s->capacity = capacity;
}

static void sliding_push(SlidingStats *s, float v) {
  if (s->count < s->capacity) {
    s->sum += v;
    s->sumSq += (double)v * v;
    s->count++;
  } else {
    float old = s->buf[s->idx];
    s->sum += v - old;
    s->sumSq += (double)v * v - (double)old * old;
  }
  s->buf[s->idx] = v;
  s->idx = (s->idx + 1) % s->capacity;
}

static float sliding_mean(const SlidingStats *s) {
  return s->count ? (float)(s->sum / s->count) : 0.0f;
}

static float sliding_rms(const SlidingStats *s) {
  if (!s->count) return 0.0f;
  double meanSq = s->sumSq / s->count;
  return sqrtf(meanSq < 0.0 ? 0.0f : (float)meanSq);
}

/* ---- One detector chain (bandpass -> RMS) per axis-triplet. One instance
   for gyro, one for accel; identical gating logic, different units/floors/
   ceilings. -------------------------------------------------------------- */
static void channel_init(TremorChannel *ch) {
  bandpass_init(&ch->bpX); bandpass_init(&ch->bpY); bandpass_init(&ch->bpZ);
  sliding_init(&ch->rmsX, TREMOR_RMS_WINDOW_SAMPLES);
  sliding_init(&ch->rmsY, TREMOR_RMS_WINDOW_SAMPLES);
  sliding_init(&ch->rmsZ, TREMOR_RMS_WINDOW_SAMPLES);
  sliding_init(&ch->shortX, SHORT_WINDOW_SAMPLES);
  sliding_init(&ch->shortY, SHORT_WINDOW_SAMPLES);
  sliding_init(&ch->shortZ, SHORT_WINDOW_SAMPLES);
  ch->calibrating = true;
  ch->calibStarted = false;
  ch->calibStartMs = 0;
  ch->calibCount = 0;
  ch->adaptiveThreshold = 0.0f;
}

/* calibCount never exceeds TREMOR_CALIB_MAX_SAMPLES (~100) - insertion
   sort is plenty fast for that, and this only runs once per channel at
   boot. */
static void sort_floats(float *arr, int n) {
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
    arr[j + 1] = key;
  }
}

static ChannelResult process_channel(TremorChannel *ch, float x, float y, float z,
                                      uint32_t now_ms, float absFloor, float absCeiling) {
  float bx = bandpass_process(&ch->bpX, x);
  float by = bandpass_process(&ch->bpY, y);
  float bz = bandpass_process(&ch->bpZ, z);

  sliding_push(&ch->rmsX, bx); sliding_push(&ch->rmsY, by); sliding_push(&ch->rmsZ, bz);
  sliding_push(&ch->shortX, bx); sliding_push(&ch->shortY, by); sliding_push(&ch->shortZ, bz);

  float rx = sliding_rms(&ch->rmsX), ry = sliding_rms(&ch->rmsY), rz = sliding_rms(&ch->rmsZ);
  float combinedRms = sqrtf(rx * rx + ry * ry + rz * rz);
  float srx = sliding_rms(&ch->shortX), sry = sliding_rms(&ch->shortY), srz = sliding_rms(&ch->shortZ);
  float shortCombinedRms = sqrtf(srx * srx + sry * sry + srz * srz);

  float dominantBp, domRms, domMean;
  if (ry >= rx && ry >= rz) { dominantBp = by; domRms = ry; domMean = sliding_mean(&ch->rmsY); }
  else if (rz >= rx)        { dominantBp = bz; domRms = rz; domMean = sliding_mean(&ch->rmsZ); }
  else                      { dominantBp = bx; domRms = rx; domMean = sliding_mean(&ch->rmsX); }

  if (ch->calibrating && ch->rmsX.count == ch->rmsX.capacity) {
    if (!ch->calibStarted) { ch->calibStarted = true; ch->calibStartMs = now_ms; }
    if (ch->calibCount < TREMOR_CALIB_MAX_SAMPLES) ch->calibSamples[ch->calibCount++] = combinedRms;

    if (now_ms - ch->calibStartMs >= CALIBRATION_MS) {
      float sorted[TREMOR_CALIB_MAX_SAMPLES];
      memcpy(sorted, ch->calibSamples, sizeof(float) * ch->calibCount);
      sort_floats(sorted, ch->calibCount);

      int idx = (int)(CALIB_PERCENTILE * ch->calibCount);
      if (idx > ch->calibCount - 1) idx = ch->calibCount - 1;
      if (idx < 0) idx = 0;

      float threshold = sorted[idx] * CALIB_MARGIN;
      if (threshold < absFloor) threshold = absFloor;
      if (threshold > absCeiling) threshold = absCeiling;
      ch->adaptiveThreshold = threshold;
      ch->calibrating = false;
    }
  }

  ChannelResult res;
  res.combinedRms = combinedRms;
  res.dominantBp = dominantBp;
  res.thresholdRatio = ch->calibrating ? -1.0f : combinedRms / ch->adaptiveThreshold;
  res.passesAllGates = false;

  if (!ch->calibrating) {
    bool isOscillatory = domRms < 1e-9f || fabsf(domMean) / domRms < NET_DISPLACEMENT_RATIO_THRESHOLD;
    bool isEnergyAboveThreshold = combinedRms > ch->adaptiveThreshold;
    bool isSustained = combinedRms < 1e-9f || shortCombinedRms >= SUSTAIN_RATIO_THRESHOLD * combinedRms;
    res.passesAllGates = isEnergyAboveThreshold && isOscillatory && isSustained;
  }
  return res;
}

/* ---- Zero-crossing frequency estimate on whichever axis currently leads -- */
static int8_t s_lastCrossSign = 0; /* 0 = uninitialised */
static uint32_t s_lastCrossTimeMs = 0;
static float s_crossingIntervals[TREMOR_MAX_CROSSING_SAMPLES];
static int s_crossingCount = 0, s_crossingHead = 0;
static float s_tremorFreqHz = 0.0f;

static void update_zero_crossing(float value, uint32_t now_ms) {
  int8_t sign = value >= 0.0f ? 1 : -1;
  if (s_lastCrossSign == 0) {
    s_lastCrossSign = sign;
    s_lastCrossTimeMs = now_ms;
    return;
  }
  if (sign != s_lastCrossSign) {
    float interval = (now_ms - s_lastCrossTimeMs) / 1000.0f;
    if (interval > MIN_CROSSING_INTERVAL_S && interval < MAX_CROSSING_INTERVAL_S) {
      s_crossingIntervals[s_crossingHead] = interval;
      s_crossingHead = (s_crossingHead + 1) % TREMOR_MAX_CROSSING_SAMPLES;
      if (s_crossingCount < TREMOR_MAX_CROSSING_SAMPLES) s_crossingCount++;

      float sum = 0.0f;
      for (int i = 0; i < s_crossingCount; i++) sum += s_crossingIntervals[i];
      float avgInterval = sum / s_crossingCount;
      s_tremorFreqHz = 1.0f / (2.0f * avgInterval); /* a full period is two zero-crossings */
    }
    s_lastCrossTimeMs = now_ms;
    s_lastCrossSign = sign;
  }
}

/* ---- Top-level detector state -------------------------------------------- */
static TremorChannel s_gyroCh, s_accelCh;
static bool s_tremorActive = false;
static bool s_aboveActive = false, s_belowActive = false;
static uint32_t s_aboveSinceMs = 0, s_belowSinceMs = 0;

static void update_hysteresis(bool passesAllGates, uint32_t now_ms) {
  if (passesAllGates) {
    s_belowActive = false;
    if (!s_aboveActive) { s_aboveActive = true; s_aboveSinceMs = now_ms; }
    if (!s_tremorActive && (now_ms - s_aboveSinceMs) >= PERSIST_ON_MS) s_tremorActive = true;
  } else {
    s_aboveActive = false;
    if (!s_belowActive) { s_belowActive = true; s_belowSinceMs = now_ms; }
    if (s_tremorActive && (now_ms - s_belowSinceMs) >= PERSIST_OFF_MS) s_tremorActive = false;
  }
}

void tremor_detect_init() {
  channel_init(&s_gyroCh);
  channel_init(&s_accelCh);

  s_lastCrossSign = 0;
  s_lastCrossTimeMs = 0;
  s_crossingCount = 0;
  s_crossingHead = 0;
  s_tremorFreqHz = 0.0f;

  s_tremorActive = false;
  s_aboveActive = false;
  s_belowActive = false;
}

TremorResult tremor_detect_update(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   uint32_t now_ms) {
  ChannelResult gRes = process_channel(&s_gyroCh, gx, gy, gz, now_ms, GYRO_ABS_FLOOR_DPS, GYRO_ABS_CEILING_DPS);
  ChannelResult aRes = process_channel(&s_accelCh, ax, ay, az, now_ms, ACCEL_ABS_FLOOR_G, ACCEL_ABS_CEILING_G);

  bool calibrating = s_gyroCh.calibrating || s_accelCh.calibrating;
  bool passesAllGates = !calibrating && (gRes.passesAllGates || aRes.passesAllGates);
  update_hysteresis(passesAllGates, now_ms);

  bool gyroLeads = gRes.thresholdRatio >= aRes.thresholdRatio;
  ChannelResult *leadRes = gyroLeads ? &gRes : &aRes;
  TremorChannel *leadCh = gyroLeads ? &s_gyroCh : &s_accelCh;
  update_zero_crossing(leadRes->dominantBp, now_ms);

  TremorResult out;
  out.active = s_tremorActive;
  out.lead_channel = gyroLeads ? 0 : 1;
  out.rms = calibrating ? 0.0f : leadRes->combinedRms;
  out.threshold = calibrating ? 0.0f : leadCh->adaptiveThreshold;
  out.freq_hz = s_tremorActive ? s_tremorFreqHz : 0.0f;
  return out;
}
