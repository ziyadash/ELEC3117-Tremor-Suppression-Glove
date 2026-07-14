// ============================================================
// Tremor detection (toggled, off by default)
// ============================================================
// Adapted from a classical EMG-based design (bandpass -> RMS envelope ->
// amplitude threshold with hysteresis) to work on accel-only IMU data,
// since this build has no EMG channel. Two substitutions from that design:
//  - no antagonist-alternation check (needs two independent EMG channels)
//  - frequency is estimated via zero-crossing counting on the dominant
//    axis, not FFT (a 256-point FFT needs seconds of data to resolve at
//    our ~50Hz packet rate - too slow to feel "live")
import { tremorScope, drawTrace, pushHistory, resizeScopes } from './scopes.js';
import { stripMat } from './scene.js';

const tremorToggleBtn = document.getElementById('tremorToggleBtn');

let tremorDetectionEnabled = false;

const TREMOR_BAND_LOW_HZ = 3;
const TREMOR_BAND_HIGH_HZ = 8;
const TREMOR_NOMINAL_FS = 50; // matches the firmware's ~50Hz notify rate
// Both attack and release are fast now - just enough smoothing on each side
// to keep a single noisy sample from flipping the state, not enough to add
// any perceptible lag in either direction.
const TREMOR_ENVELOPE_ATTACK_TAU = 0.03; // seconds; fast rise once real tremor-band energy shows up
const TREMOR_ENVELOPE_RELEASE_TAU = 0.03; // seconds; fast falloff once motion stops
const TREMOR_AMPLITUDE_THRESHOLD_G = 0.03; // tune experimentally per glove/mount
const TREMOR_ON_HOLD_MS = 40; // just enough to debounce a single packet, not a perceptible delay
const TREMOR_OFF_HOLD_MS = 0; // drop back to idle as soon as the envelope reads quiet, no added delay
const TREMOR_RMS_SCOPE_FULL_SCALE_G = 0.15;
const MIN_CROSSING_INTERVAL_S = 0.02; // ignore implausible tremor rates (>25Hz)
const MAX_CROSSING_INTERVAL_S = 1.0;  // ignore implausible tremor rates (<0.5Hz)
const MAX_CROSSING_SAMPLES = 6;

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

// RBJ audio-EQ-cookbook biquad design at Butterworth Q (1/sqrt(2)).
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
  const hp = designButterworthBiquad('highpass', TREMOR_BAND_LOW_HZ, TREMOR_NOMINAL_FS);
  const lp = designButterworthBiquad('lowpass', TREMOR_BAND_HIGH_HZ, TREMOR_NOMINAL_FS);
  return { process(v) { return lp.process(hp.process(v)); } };
}

let tremorBpX, tremorBpY, tremorBpZ;
let tremorEnvX2 = 0, tremorEnvY2 = 0, tremorEnvZ2 = 0;
let tremorActive = false;
let tremorAboveSince = null, tremorBelowSince = null;
let tremorLastCrossSign = null, tremorLastCrossTime = null;
let tremorCrossingIntervals = [];
let tremorFreqHz = 0;

function updateTremorZeroCrossing(value, now) {
  const sign = value >= 0 ? 1 : -1;
  if (tremorLastCrossSign === null) {
    tremorLastCrossSign = sign;
    tremorLastCrossTime = now;
    return;
  }
  if (sign !== tremorLastCrossSign) {
    const interval = (now - tremorLastCrossTime) / 1000;
    if (interval > MIN_CROSSING_INTERVAL_S && interval < MAX_CROSSING_INTERVAL_S) {
      tremorCrossingIntervals.push(interval);
      if (tremorCrossingIntervals.length > MAX_CROSSING_SAMPLES) tremorCrossingIntervals.shift();
      const avgInterval = tremorCrossingIntervals.reduce((a, b) => a + b, 0) / tremorCrossingIntervals.length;
      tremorFreqHz = 1 / (2 * avgInterval); // a full period is two zero-crossings
    }
    tremorLastCrossTime = now;
    tremorLastCrossSign = sign;
  }
}

function updateTremorHysteresis(isTremor, now) {
  if (isTremor) {
    tremorBelowSince = null;
    if (tremorAboveSince === null) tremorAboveSince = now;
    if (!tremorActive && now - tremorAboveSince >= TREMOR_ON_HOLD_MS) tremorActive = true;
  } else {
    tremorAboveSince = null;
    if (tremorBelowSince === null) tremorBelowSince = now;
    if (tremorActive && now - tremorBelowSince >= TREMOR_OFF_HOLD_MS) tremorActive = false;
  }
}

// One-pole envelope on the squared (power) signal, with a slower time
// constant while energy is rising (attack) and a faster one while it's
// falling (release) - see the constants above for why.
function updateEnvelope2(prevValue2, targetValue2, dt) {
  const tau = targetValue2 > prevValue2 ? TREMOR_ENVELOPE_ATTACK_TAU : TREMOR_ENVELOPE_RELEASE_TAU;
  const alpha = 1 - Math.exp(-dt / tau);
  return prevValue2 + alpha * (targetValue2 - prevValue2);
}

export function runTremorDetection(ax, ay, az, now, dt) {
  const bpX = tremorBpX.process(ax);
  const bpY = tremorBpY.process(ay);
  const bpZ = tremorBpZ.process(az);

  tremorEnvX2 = updateEnvelope2(tremorEnvX2, bpX * bpX, dt);
  tremorEnvY2 = updateEnvelope2(tremorEnvY2, bpY * bpY, dt);
  tremorEnvZ2 = updateEnvelope2(tremorEnvZ2, bpZ * bpZ, dt);
  const tremorRms = Math.sqrt(tremorEnvX2 + tremorEnvY2 + tremorEnvZ2);

  // Track zero-crossings on whichever axis currently carries the most
  // tremor-band energy (mirrors a "dominant tremor axis" idea used
  // elsewhere for per-actuator drive weighting).
  let dominantBp = bpX, dominantEnv = tremorEnvX2;
  if (tremorEnvY2 > dominantEnv) { dominantBp = bpY; dominantEnv = tremorEnvY2; }
  if (tremorEnvZ2 > dominantEnv) { dominantBp = bpZ; dominantEnv = tremorEnvZ2; }
  updateTremorZeroCrossing(dominantBp, now);

  updateTremorHysteresis(tremorRms > TREMOR_AMPLITUDE_THRESHOLD_G, now);

  pushHistory(tremorScope.history, THREE.MathUtils.clamp(tremorRms / TREMOR_RMS_SCOPE_FULL_SCALE_G, 0, 1));
  drawTrace(tremorScope);

  const stateEl = document.getElementById('tremorState');
  stateEl.textContent = tremorActive ? 'ACTIVE' : 'idle';
  stateEl.classList.toggle('tremor-active', tremorActive);
  document.getElementById('tremorFreq').textContent = tremorActive ? tremorFreqHz.toFixed(1) + ' Hz' : '--';
  document.getElementById('tremorRms').textContent = tremorRms.toFixed(3);

  // Tint the board's strip pink while active - stands in for "this is
  // when the actuators would fire" since there's no haptic driver here.
  stripMat.color.setHex(tremorActive ? 0xec4899 : 0x3b82f6);
  stripMat.emissive.setHex(tremorActive ? 0x8a1550 : 0x1e3a6b);
}

export function resetTremorState() {
  tremorBpX = makeTremorBandpass();
  tremorBpY = makeTremorBandpass();
  tremorBpZ = makeTremorBandpass();
  tremorEnvX2 = tremorEnvY2 = tremorEnvZ2 = 0;
  tremorActive = false;
  tremorAboveSince = null;
  tremorBelowSince = null;
  tremorLastCrossSign = null;
  tremorLastCrossTime = null;
  tremorCrossingIntervals = [];
  tremorFreqHz = 0;
  tremorScope.history = [];
  drawTrace(tremorScope);

  stripMat.color.setHex(0x3b82f6);
  stripMat.emissive.setHex(0x1e3a6b);
  document.getElementById('tremorState').textContent = 'idle';
  document.getElementById('tremorState').classList.remove('tremor-active');
  document.getElementById('tremorFreq').textContent = '--';
  document.getElementById('tremorRms').textContent = '0.000';
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
