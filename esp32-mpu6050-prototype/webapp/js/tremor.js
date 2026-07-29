// ============================================================
// Tremor detection (toggled, off by default)
// ============================================================
// Dual-channel: tremor can show up as rotational energy (gyro) or
// translational energy (accel) depending on the motion, so this runs two
// independent, identically-structured detector chains and calls it
// tremor if *either* one's gates pass. Accel here is already
// gravity-compensated linear accel from the DMP, not raw accel.
//
// Pipeline, per channel, per axis (x, y, z), every packet:
//   1. 3-12Hz Butterworth bandpass (rejects slow voluntary motion and
//      high-freq noise; covers essential/Parkinsonian rest tremor up to
//      low physiological tremor)
//   2. Sliding-window RMS, combined across axes into one magnitude
//   3. Combined RMS is checked against an adaptive threshold, calibrated
//      per-channel from a resting baseline captured after the toggle is
//      enabled (85th percentile of resting RMS * a margin, clamped to a
//      sane floor/ceiling - percentile instead of mean+std because a
//      calibration window that isn't perfectly still shouldn't be able to
//      blow the threshold up via a couple of outlier samples)
//
// A channel counts as tremor once all of these hold:
//   - energy: combined RMS exceeds that channel's threshold
//   - oscillatory: the dominant axis's mean over the window is small
//     relative to its RMS (a zero-mean oscillation nets ~0 displacement;
//     a ramp or a single jerk/spike nets close to its own RMS)
//   - sustained: a short recent-energy window stays close to the full
//     window's RMS (a genuine oscillation holds steady; a bandpass
//     filter's ringdown from a one-off jerk decays within the window)
//
// gyroPasses || accelPasses then has to hold for a short dwell time
// before the shared state flips to ACTIVE, and clear for a shorter dwell
// before dropping back to idle.
//
// Frequency is a simple zero-crossing count on whichever axis is
// currently carrying the most energy - display only, doesn't gate
// anything.
import { tremorScope, drawTrace, pushHistory, resizeScopes } from './scopes.js';
import { stripMat } from './scene.js';

const tremorToggleBtn = document.getElementById('tremorToggleBtn');

let tremorDetectionEnabled = false;

// ---- Tunables ----------------------------------------------------------
const TREMOR_BAND_LOW_HZ = 3;
const TREMOR_BAND_HIGH_HZ = 12;
const FS_HZ = 50; // matches the firmware's paced BLE notify rate (SAMPLE_PERIOD_MS)
const RMS_WINDOW_SEC = 0.6;
const SUSTAIN_WINDOW_SEC = 0.15;
const SUSTAIN_RATIO_THRESHOLD = 0.55; // recent RMS must stay at least this fraction of the full-window RMS
const CALIBRATION_MS = 2000;
const CALIB_PERCENTILE = 0.85;
const CALIB_MARGIN = 1.6;
const GYRO_ABS_FLOOR_DPS = 1.5;
const GYRO_ABS_CEILING_DPS = 15;
const ACCEL_ABS_FLOOR_G = 0.03;
const ACCEL_ABS_CEILING_G = 0.12;
const NET_DISPLACEMENT_RATIO_THRESHOLD = 0.45; // |mean|/rms on the dominant axis; near 0 = oscillation, near 1 = ramp/jerk
const PERSIST_ON_MS = 300;
const PERSIST_OFF_MS = 150;
const SCOPE_FULL_SCALE_THRESHOLD_MULTIPLE = 2; // scope trace fills at this many multiples of threshold
const MIN_CROSSING_INTERVAL_S = 0.02; // ignore implausible tremor rates (>25Hz)
const MAX_CROSSING_INTERVAL_S = 1.0;  // ignore implausible tremor rates (<0.5Hz)
const MAX_CROSSING_SAMPLES = 6;

// ---- Biquad bandpass (RBJ audio-EQ-cookbook, Butterworth Q) ------------
function makeBiquad() {
  return {
    b0: 1, b1: 0, b2: 0, a1: 0, a2: 0,
    x1: 0, x2: 0, y1: 0, y2: 0,
    process(x) {
      const y = this.b0 * x + this.b1 * this.x1 + this.b2 * this.x2
        - this.a1 * this.y1 - this.a2 * this.y2;
      this.x2 = this.x1; this.x1 = x;
      this.y2 = this.y1; this.y1 = y;
      return y;
    }
  };
}

function designButterworthBiquad(type, fc, fs) {
  const w0 = 2 * Math.PI * fc / fs;
  const cosw0 = Math.cos(w0), sinw0 = Math.sin(w0);
  const alpha = sinw0 / (2 * Math.SQRT1_2);
  const a0 = 1 + alpha;
  const bq = makeBiquad();
  if (type === 'lowpass') {
    bq.b0 = ((1 - cosw0) / 2) / a0;
    bq.b1 = (1 - cosw0) / a0;
    bq.b2 = ((1 - cosw0) / 2) / a0;
  } else {
    bq.b0 = ((1 + cosw0) / 2) / a0;
    bq.b1 = (-(1 + cosw0)) / a0;
    bq.b2 = ((1 + cosw0) / 2) / a0;
  }
  bq.a1 = (-2 * cosw0) / a0;
  bq.a2 = (1 - alpha) / a0;
  return bq;
}

function makeTremorBandpass() {
  const hp = designButterworthBiquad('highpass', TREMOR_BAND_LOW_HZ, FS_HZ);
  const lp = designButterworthBiquad('lowpass', TREMOR_BAND_HIGH_HZ, FS_HZ);
  return { process(v) { return lp.process(hp.process(v)); } };
}

// ---- Sliding-window mean/RMS (O(1) per sample) --------------------------
function makeSlidingStats(sec) {
  const capacity = Math.max(4, Math.round(sec * FS_HZ));
  return {
    capacity,
    buf: new Float64Array(capacity),
    idx: 0, count: 0, sum: 0, sumSq: 0,
    push(v) {
      if (this.count < this.capacity) {
        this.sum += v;
        this.sumSq += v * v;
        this.count++;
      } else {
        const old = this.buf[this.idx];
        this.sum += v - old;
        this.sumSq += v * v - old * old;
      }
      this.buf[this.idx] = v;
      this.idx = (this.idx + 1) % this.capacity;
    },
    get mean() { return this.count ? this.sum / this.count : 0; },
    get rms() { return this.count ? Math.sqrt(Math.max(0, this.sumSq / this.count)) : 0; }
  };
}

// ---- One detector chain (bandpass -> RMS) per axis-triplet. Instantiated
// once for gyro, once for accel; identical gating logic runs over both,
// just with different units/floors/ceilings. -----------------------------
function makeChannel() {
  return {
    bpX: makeTremorBandpass(), bpY: makeTremorBandpass(), bpZ: makeTremorBandpass(),
    rmsX: makeSlidingStats(RMS_WINDOW_SEC), rmsY: makeSlidingStats(RMS_WINDOW_SEC), rmsZ: makeSlidingStats(RMS_WINDOW_SEC),
    shortX: makeSlidingStats(SUSTAIN_WINDOW_SEC), shortY: makeSlidingStats(SUSTAIN_WINDOW_SEC), shortZ: makeSlidingStats(SUSTAIN_WINDOW_SEC),
    calibrating: true,
    calibrationStartTime: null,
    calibSamples: [],
    adaptiveThreshold: 0
  };
}

// Runs one packet through one channel: filters, updates stats, calibrates
// if still warming up, evaluates the gates.
function processChannel(ch, x, y, z, now, absFloor, absCeiling) {
  const bx = ch.bpX.process(x), by = ch.bpY.process(y), bz = ch.bpZ.process(z);
  ch.rmsX.push(bx); ch.rmsY.push(by); ch.rmsZ.push(bz);
  ch.shortX.push(bx); ch.shortY.push(by); ch.shortZ.push(bz);

  const axisRms = { x: ch.rmsX.rms, y: ch.rmsY.rms, z: ch.rmsZ.rms };
  const combinedRms = Math.sqrt(axisRms.x ** 2 + axisRms.y ** 2 + axisRms.z ** 2);
  const shortCombinedRms = Math.sqrt(ch.shortX.rms ** 2 + ch.shortY.rms ** 2 + ch.shortZ.rms ** 2);
  const dominantAxis = axisRms.y >= axisRms.x && axisRms.y >= axisRms.z ? 'y'
    : axisRms.z >= axisRms.x ? 'z' : 'x';
  const dominantBp = { x: bx, y: by, z: bz }[dominantAxis];

  if (ch.calibrating && ch.rmsX.count === ch.rmsX.capacity) {
    if (ch.calibrationStartTime === null) ch.calibrationStartTime = now;
    ch.calibSamples.push(combinedRms);
    if (now - ch.calibrationStartTime >= CALIBRATION_MS) {
      const sorted = ch.calibSamples.slice().sort((a, b) => a - b);
      const percentileValue = sorted[Math.min(sorted.length - 1, Math.floor(CALIB_PERCENTILE * sorted.length))];
      ch.adaptiveThreshold = Math.min(absCeiling, Math.max(absFloor, percentileValue * CALIB_MARGIN));
      ch.calibrating = false;
      ch.calibSamples = null;
    }
  }

  let passesAllGates = false;
  if (!ch.calibrating) {
    const domStats = { x: ch.rmsX, y: ch.rmsY, z: ch.rmsZ }[dominantAxis];
    const isOscillatory = domStats.rms < 1e-9 || Math.abs(domStats.mean) / domStats.rms < NET_DISPLACEMENT_RATIO_THRESHOLD;
    const isEnergyAboveThreshold = combinedRms > ch.adaptiveThreshold;
    // Rejects a single jerk/spike: a bandpass filter rings for a bit after
    // any hard transient, and that ringdown can look oscillatory even
    // though the underlying motion wasn't. A genuine sustained oscillation
    // keeps its recent energy close to the full-window average; a
    // decaying ringdown's recent energy drops well below it.
    const isSustained = combinedRms < 1e-9 || shortCombinedRms >= SUSTAIN_RATIO_THRESHOLD * combinedRms;
    passesAllGates = isEnergyAboveThreshold && isOscillatory && isSustained;
  }

  return {
    passesAllGates,
    combinedRms,
    dominantBp,
    thresholdRatio: ch.calibrating ? -1 : combinedRms / ch.adaptiveThreshold
  };
}

// ---- Zero-crossing frequency estimate on whichever axis currently leads --
let lastCrossSign = null, lastCrossTime = null;
let crossingIntervals = [];
let tremorFreqHz = 0;

function updateZeroCrossing(value, now) {
  const sign = value >= 0 ? 1 : -1;
  if (lastCrossSign === null) {
    lastCrossSign = sign;
    lastCrossTime = now;
    return;
  }
  if (sign !== lastCrossSign) {
    const interval = (now - lastCrossTime) / 1000;
    if (interval > MIN_CROSSING_INTERVAL_S && interval < MAX_CROSSING_INTERVAL_S) {
      crossingIntervals.push(interval);
      if (crossingIntervals.length > MAX_CROSSING_SAMPLES) crossingIntervals.shift();
      const avgInterval = crossingIntervals.reduce((a, b) => a + b, 0) / crossingIntervals.length;
      tremorFreqHz = 1 / (2 * avgInterval); // a full period is two zero-crossings
    }
    lastCrossTime = now;
    lastCrossSign = sign;
  }
}

// ---- Detector state -------------------------------------------------------
let gyroCh, accelCh;
let tremorActive = false;
let aboveSince = null, belowSince = null;

function updateTremorHysteresis(passesAllGates, now) {
  if (passesAllGates) {
    belowSince = null;
    if (aboveSince === null) aboveSince = now;
    if (!tremorActive && now - aboveSince >= PERSIST_ON_MS) tremorActive = true;
  } else {
    aboveSince = null;
    if (belowSince === null) belowSince = now;
    if (tremorActive && now - belowSince >= PERSIST_OFF_MS) tremorActive = false;
  }
}

export function runTremorDetection(gx, gy, gz, ax, ay, az, now) {
  const gRes = processChannel(gyroCh, gx, gy, gz, now, GYRO_ABS_FLOOR_DPS, GYRO_ABS_CEILING_DPS);
  const aRes = processChannel(accelCh, ax, ay, az, now, ACCEL_ABS_FLOOR_G, ACCEL_ABS_CEILING_G);

  const calibrating = gyroCh.calibrating || accelCh.calibrating;
  const passesAllGates = !calibrating && (gRes.passesAllGates || aRes.passesAllGates);
  updateTremorHysteresis(passesAllGates, now);

  // Whichever channel is currently further above its own threshold drives
  // the display (rms/threshold/units) and the frequency estimate.
  const gyroLeads = gRes.thresholdRatio >= aRes.thresholdRatio;
  const leadChannel = gyroLeads ? gyroCh : accelCh;
  const leadRes = gyroLeads ? gRes : aRes;
  const leadUnit = gyroLeads ? '°/s' : 'g';
  updateZeroCrossing(leadRes.dominantBp, now);

  const scopeValue = calibrating ? 0
    : leadRes.combinedRms / (leadChannel.adaptiveThreshold * SCOPE_FULL_SCALE_THRESHOLD_MULTIPLE);
  pushHistory(tremorScope.history, THREE.MathUtils.clamp(scopeValue, 0, 1));
  drawTrace(tremorScope);

  const stateEl = document.getElementById('tremorState');
  stateEl.textContent = calibrating ? 'calibrating…' : (tremorActive ? 'ACTIVE' : 'idle');
  stateEl.classList.toggle('tremor-active', tremorActive);
  document.getElementById('tremorFreq').textContent = tremorActive ? tremorFreqHz.toFixed(1) + ' Hz' : '--';
  document.getElementById('tremorRms').textContent = calibrating ? '--' : leadRes.combinedRms.toFixed(3) + ' ' + leadUnit;
  document.getElementById('tremorThreshold').textContent = calibrating ? '--' : leadChannel.adaptiveThreshold.toFixed(3) + ' ' + leadUnit;
  document.getElementById('tremorChannel').textContent = calibrating ? '--' : (gyroLeads ? 'gyro' : 'accel');

  // Tint the board's strip pink while active - stands in for "this is
  // when the actuators would fire" since there's no haptic driver here.
  stripMat.color.setHex(tremorActive ? 0xec4899 : 0x3b82f6);
  stripMat.emissive.setHex(tremorActive ? 0x8a1550 : 0x1e3a6b);
}

export function resetTremorState() {
  gyroCh = makeChannel();
  accelCh = makeChannel();

  lastCrossSign = null;
  lastCrossTime = null;
  crossingIntervals = [];
  tremorFreqHz = 0;

  tremorActive = false;
  aboveSince = null;
  belowSince = null;

  tremorScope.history = [];
  drawTrace(tremorScope);

  stripMat.color.setHex(0x3b82f6);
  stripMat.emissive.setHex(0x1e3a6b);
  document.getElementById('tremorState').textContent = 'calibrating…';
  document.getElementById('tremorState').classList.remove('tremor-active');
  document.getElementById('tremorFreq').textContent = '--';
  document.getElementById('tremorRms').textContent = '--';
  document.getElementById('tremorThreshold').textContent = '--';
  document.getElementById('tremorChannel').textContent = '--';
}
resetTremorState();

function setTremorDetectionEnabled(enabled) {
  tremorDetectionEnabled = enabled;
  document.body.classList.toggle('tremor-on', enabled);
  tremorToggleBtn.textContent = 'Tremor detection: ' + (enabled ? 'on' : 'off');
  tremorToggleBtn.classList.toggle('active', enabled);
  resetTremorState();
  if (enabled) resizeScopes(); // the scope column was display:none, so it had no real size yet
}

export function isTremorDetectionEnabled() {
  return tremorDetectionEnabled;
}

tremorToggleBtn.addEventListener('click', () => setTremorDetectionEnabled(!tremorDetectionEnabled));
