// ============================================================
// Tremor status display
// ============================================================
// Detection now runs on-device (tremor_glove/tremor_detect.ino — a direct
// port of what used to live here: dual-channel gyro+accel, 3-gate,
// adaptively-calibrated). This module no longer computes anything; it
// just draws whatever the firmware already decided into the UI, parsed
// out of the BLE packet by main.js.
import { tremorScope, drawTrace, pushHistory, resizeScopes } from './scopes.js';
import { stripMat } from './scene.js';

const tremorToggleBtn = document.getElementById('tremorToggleBtn');
let tremorDetectionEnabled = false;

// Scope trace fills at this many multiples of threshold.
const SCOPE_FULL_SCALE_THRESHOLD_MULTIPLE = 2;

// threshold <= 0 is how the firmware signals "still calibrating" (it only
// starts publishing a real threshold once the ~2.6s boot calibration
// finishes) — no separate flag needed on the wire for that.
export function updateTremorDisplay(active, leadChannel, rms, threshold, freqHz) {
  if (!tremorDetectionEnabled) return;

  const calibrating = threshold <= 0;
  const leadUnit = leadChannel === 0 ? '°/s' : 'g';

  const scopeValue = calibrating ? 0 : rms / (threshold * SCOPE_FULL_SCALE_THRESHOLD_MULTIPLE);
  pushHistory(tremorScope.history, THREE.MathUtils.clamp(scopeValue, 0, 1));
  drawTrace(tremorScope);

  const stateEl = document.getElementById('tremorState');
  stateEl.textContent = calibrating ? 'calibrating…' : (active ? 'ACTIVE' : 'idle');
  stateEl.classList.toggle('tremor-active', active);
  document.getElementById('tremorFreq').textContent = active ? freqHz.toFixed(1) + ' Hz' : '--';
  document.getElementById('tremorRms').textContent = calibrating ? '--' : rms.toFixed(3) + ' ' + leadUnit;
  document.getElementById('tremorThreshold').textContent = calibrating ? '--' : threshold.toFixed(3) + ' ' + leadUnit;
  document.getElementById('tremorChannel').textContent = calibrating ? '--' : (leadChannel === 0 ? 'gyro' : 'accel');

  // Tint the board's strip pink while active - stands in for "this is
  // when the actuators would fire" for anyone watching without the glove.
  stripMat.color.setHex(active ? 0xec4899 : 0x3b82f6);
  stripMat.emissive.setHex(active ? 0x8a1550 : 0x1e3a6b);
}

export function resetTremorDisplay() {
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
resetTremorDisplay();

function setTremorDetectionEnabled(enabled) {
  tremorDetectionEnabled = enabled;
  document.body.classList.toggle('tremor-on', enabled);
  tremorToggleBtn.textContent = 'Tremor detection: ' + (enabled ? 'on' : 'off');
  tremorToggleBtn.classList.toggle('active', enabled);
  resetTremorDisplay();
  if (enabled) resizeScopes(); // the scope column was display:none, so it had no real size yet
}

tremorToggleBtn.addEventListener('click', () => setTremorDetectionEnabled(!tremorDetectionEnabled));
