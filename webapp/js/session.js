// ============================================================
// Session recording — tremor + motor timeline export
// ============================================================
// Records two independent active/idle transition logs for the currently
// connected session - tremor detection state, and whether the haptic
// motors are actually driving (edge-triggered, not one sample per packet
// - both are step functions, so only transitions matter for
// reconstructing exact intervals) - and renders them as a downloadable,
// timestamped SVG timeline with one row each. Recording runs
// independently of the tremor-detection UI toggle in tremor.js - that
// toggle only controls whether the panel is *shown*, detection and
// motor drive both happen on-device regardless, so the session log
// should capture them regardless of what the toggle happens to be set
// to at any given moment.
const saveBtn = document.getElementById('saveSessionBtn');
const latencyEl = document.getElementById('tremorLatency');

let sessionStartMs = null;
let tremorEvents = []; // { tMs: elapsed ms since sessionStartMs, active }
let lastTremorActive = null;
let motorEvents = []; // { tMs, active: driveIntensity > 0 }
let lastMotorOn = null;

// Continuous traces (every sample, not just transitions) - drive the
// intensity heat-strip so the graph shows how hard the tremor is and how
// hard the motor is buzzing, not just a flat on/off block.
let tremorTrace = []; // { tMs, ratio: rms / threshold }
let motorTrace = [];  // { tMs, intensity: driveIntensity }

// Detection-latency measurement, independent of session recording: the
// webapp timestamps every BLE packet, so idle->ACTIVE transition time can
// be measured directly from consecutive packets - no firmware change or
// reflash needed. lastIdleWallClockMs is updated on every idle packet, so
// by the time a packet reports ACTIVE, the gap back to it is a faithful
// end-to-end measurement: it already includes however long the on-device
// RMS window + gates + persistence timer took to confirm real tremor,
// not just the fixed persistence constant.
let lastIdleWallClockMs = null;

export function startSession() {
  sessionStartMs = performance.now();
  tremorEvents = [];
  lastTremorActive = null;
  motorEvents = [];
  lastMotorOn = null;
  tremorTrace = [];
  motorTrace = [];
  lastIdleWallClockMs = null;
  saveBtn.disabled = true;
  latencyEl.textContent = '--';
}

// ratio = rms / threshold for the leading channel, 0 while calibrating
// (threshold <= 0) or idle. Only meaningful once >1 (that's the gate the
// firmware itself uses to flag active), so it doubles as an intensity
// signal for the graph - no separate "how strong" field needed on the wire.
export function recordTremorSample(active, ratio = 0) {
  const now = performance.now();
  const wasIdle = lastTremorActive === false;
  if (!active) {
    lastIdleWallClockMs = now;
  } else if (wasIdle && lastIdleWallClockMs !== null) {
    latencyEl.textContent = Math.round(now - lastIdleWallClockMs) + ' ms';
  }

  if (sessionStartMs === null) return;
  tremorTrace.push({ tMs: now - sessionStartMs, ratio });
  if (active === lastTremorActive) return; // only log transitions
  lastTremorActive = active;
  tremorEvents.push({ tMs: now - sessionStartMs, active });
  saveBtn.disabled = false;
}

export function recordMotorSample(driveIntensity) {
  if (sessionStartMs === null) return;
  const tMs = performance.now() - sessionStartMs;
  motorTrace.push({ tMs, intensity: driveIntensity });
  const on = driveIntensity > 0;
  if (on === lastMotorOn) return; // only log transitions
  lastMotorOn = on;
  motorEvents.push({ tMs, active: on });
  saveBtn.disabled = false;
}

function formatDuration(ms) {
  const totalSec = Math.round(ms / 1000);
  const m = Math.floor(totalSec / 60);
  const s = totalSec % 60;
  return m + 'm ' + s + 's';
}

// Turns a transition log into contiguous segments spanning [0, nowMs),
// closing the final segment at "now" so the graph reflects up to the
// moment of export, not just the last recorded transition.
function toSegments(events, nowMs) {
  const segments = [];
  for (let i = 0; i < events.length; i++) {
    const start = events[i].tMs;
    const end = i + 1 < events.length ? events[i + 1].tMs : nowMs;
    segments.push({ start, end, active: events[i].active });
  }
  return segments;
}

function clamp01(t) {
  return Math.max(0, Math.min(1, t));
}

function lerpColor(hexA, hexB, t) {
  const a = parseInt(hexA.slice(1), 16), b = parseInt(hexB.slice(1), 16);
  const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
  const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
  const r = Math.round(ar + (br - ar) * t), g = Math.round(ag + (bg - ag) * t), bl = Math.round(ab + (bb - ab) * t);
  return `#${((1 << 24) + (r << 16) + (g << 8) + bl).toString(16).slice(1)}`;
}

// Ratio 1.0 is the firmware's own active/idle gate (rms > threshold), so
// intensity is 0 right up until that crossing. 2.5 matches DRIVE_FULL_SCALE_RATIO
// in tremor_glove.ino - the point where the motor's own drive saturates - so a
// fully-saturated pink column lines up with a fully-saturated green one below it.
const TREMOR_FULL_SCALE_RATIO = 2.5;
const MOTOR_FULL_SCALE = 50; // matches DRIVE_MAX in tremor_glove.ino

function tremorIntensity(ratio) {
  if (!ratio || ratio <= 1) return 0;
  return clamp01((ratio - 1) / (TREMOR_FULL_SCALE_RATIO - 1));
}

function motorIntensity(drive) {
  return clamp01(drive / MOTOR_FULL_SCALE);
}

// Buckets a continuous trace into one column per horizontal pixel, taking
// the max intensity seen in that column - cheap single pass, and a max
// (not average) keeps brief peaks visible instead of smearing them out.
function heatColumns(trace, xFor, marginL, plotW, intensityOf) {
  const n = Math.max(1, Math.round(plotW));
  const cols = new Float32Array(n).fill(-1);
  for (const s of trace) {
    const col = Math.max(0, Math.min(n - 1, Math.floor(xFor(s.tMs) - marginL)));
    const v = intensityOf(s);
    if (v > cols[col]) cols[col] = v;
  }
  return cols;
}

function heatRowSvg(trace, xFor, marginL, plotW, y, height, baseColor, fullColor, intensityOf) {
  const cols = heatColumns(trace, xFor, marginL, plotW, intensityOf);
  const colW = plotW / cols.length;
  let bands = '';
  for (let i = 0; i < cols.length; i++) {
    const t = cols[i] < 0 ? 0 : cols[i];
    const x = marginL + i * colW;
    bands += `<rect x="${x.toFixed(1)}" y="${y}" width="${(colW + 0.5).toFixed(1)}" height="${height}" fill="${lerpColor(baseColor, fullColor, t)}"/>`;
  }
  return bands;
}

function activeMsOf(segments) {
  return segments.filter(s => s.active).reduce((sum, s) => sum + (s.end - s.start), 0);
}

function buildSessionSvg() {
  const nowMs = performance.now() - sessionStartMs;
  const totalMs = Math.max(1, nowMs);
  const startDate = new Date(Date.now() - totalMs);

  const tremorSegments = toSegments(tremorEvents, nowMs);
  const motorSegments = toSegments(motorEvents, nowMs);
  const tremorActiveMs = activeMsOf(tremorSegments);
  const motorActiveMs = activeMsOf(motorSegments);

  const W = 900, H = 260;
  const marginL = 50, marginR = 30;
  const plotW = W - marginL - marginR;
  const rowH = 44;
  const tremorY = 56, motorY = tremorY + rowH + 34;
  const xFor = (tMs) => marginL + (tMs / totalMs) * plotW;

  let ticks = '';
  const numTicks = 6;
  const tickY = motorY + rowH;
  for (let i = 0; i <= numTicks; i++) {
    const t = (totalMs / numTicks) * i;
    const x = xFor(t);
    const totalSec = Math.round(t / 1000);
    const label = Math.floor(totalSec / 60) + ':' + String(totalSec % 60).padStart(2, '0');
    ticks += `<line x1="${x.toFixed(1)}" y1="${tickY}" x2="${x.toFixed(1)}" y2="${tickY + 5}" stroke="#67717c" stroke-width="1"/>`;
    ticks += `<text x="${x.toFixed(1)}" y="${tickY + 20}" class="ts" text-anchor="middle">${label}</text>`;
  }

  const tremorPct = ((tremorActiveMs / totalMs) * 100).toFixed(1);
  const motorPct = ((motorActiveMs / totalMs) * 100).toFixed(1);

  const tremorHeat = heatRowSvg(tremorTrace, xFor, marginL, plotW, tremorY, rowH, '#dde2e7', '#ec4899', s => tremorIntensity(s.ratio));
  const motorHeat = heatRowSvg(motorTrace, xFor, marginL, plotW, motorY, rowH, '#dde2e7', '#0f9c82', s => motorIntensity(s.intensity));

  const legendGrad = (id, color) => `<linearGradient id="${id}" x1="0" y1="0" x2="1" y2="0">
<stop offset="0" stop-color="#dde2e7"/><stop offset="1" stop-color="${color}"/></linearGradient>`;

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">
<style>text{font-family:'JetBrains Mono','Courier New',monospace}.t{font-size:15px;fill:#1b232c}.ts{font-size:12px;fill:#67717c}</style>
<defs>${legendGrad('legendTremor', '#ec4899')}${legendGrad('legendMotor', '#0f9c82')}</defs>
<rect width="${W}" height="${H}" fill="#ffffff"/>
<text x="${marginL}" y="24" class="t">Tremor session — ${startDate.toLocaleString()}</text>
<rect x="${W - 190}" y="14" width="40" height="11" fill="url(#legendTremor)"/>
<text x="${W - 146}" y="23" class="ts">tremor intensity</text>
<rect x="${W - 190}" y="30" width="40" height="11" fill="url(#legendMotor)"/>
<text x="${W - 146}" y="39" class="ts">motor intensity</text>
<text x="${marginL}" y="${tremorY - 8}" class="ts">tremor</text>
<rect x="${marginL}" y="${tremorY}" width="${plotW}" height="${rowH}" fill="#dde2e7"/>
${tremorHeat}
<rect x="${marginL}" y="${tremorY}" width="${plotW}" height="${rowH}" fill="none" stroke="#dde2e7" stroke-width="1"/>
<text x="${marginL}" y="${motorY - 8}" class="ts">motor</text>
<rect x="${marginL}" y="${motorY}" width="${plotW}" height="${rowH}" fill="#dde2e7"/>
${motorHeat}
<rect x="${marginL}" y="${motorY}" width="${plotW}" height="${rowH}" fill="none" stroke="#dde2e7" stroke-width="1"/>
${ticks}
<text x="${marginL}" y="${H - 12}" class="ts">duration ${formatDuration(totalMs)} · tremor active ${formatDuration(tremorActiveMs)} (${tremorPct}%) · motor on ${formatDuration(motorActiveMs)} (${motorPct}%)</text>
</svg>`;

  return { svg, startDate };
}

export function saveSessionGraph() {
  if (sessionStartMs === null || (tremorEvents.length === 0 && motorEvents.length === 0)) return;

  const { svg, startDate } = buildSessionSvg();
  const blob = new Blob([svg], { type: 'image/svg+xml' });
  const url = URL.createObjectURL(blob);
  const isoStamp = startDate.toISOString().replace(/[:.]/g, '-');

  const a = document.createElement('a');
  a.href = url;
  a.download = `tremor-session-${isoStamp}.svg`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  // The browser reads the blob URL asynchronously to actually save the
  // file - revoking it in the same tick as click() races that read and
  // can silently produce a truncated or missing download. Give it a
  // moment first.
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}

saveBtn.addEventListener('click', saveSessionGraph);
