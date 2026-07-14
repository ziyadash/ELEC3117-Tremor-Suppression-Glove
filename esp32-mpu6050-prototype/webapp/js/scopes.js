// ============================================================
// Scopes (rolling traces)
// ============================================================
export const MAX_HISTORY = 240;

export const rollScope = {
  canvas: document.getElementById('scope-canvas-roll'),
  history: [],
  baselineFrac: 0.5, // reference line centered; roll swings +/-180deg
  color: '#4de3c1'
};
export const accelScope = {
  canvas: document.getElementById('scope-canvas-accel'),
  history: [],
  baselineFrac: 0.96, // reference line near the bottom; magnitude is >= 0
  color: '#e3a34d'
};
export const tremorScope = {
  canvas: document.getElementById('scope-canvas-tremor'),
  history: [],
  baselineFrac: 0.96,
  color: '#e3644d'
};
const ALL_SCOPES = [rollScope, accelScope, tremorScope];
ALL_SCOPES.forEach(s => { s.ctx = s.canvas.getContext('2d'); });

export function resizeScopes() {
  // Hidden (tremor-only, toggled off) canvases resize to 0x0 harmlessly;
  // resizeScopes() is called again when the toggle turns them visible.
  ALL_SCOPES.forEach(s => {
    s.canvas.width = s.canvas.clientWidth * devicePixelRatio;
    s.canvas.height = s.canvas.clientHeight * devicePixelRatio;
  });
}
window.addEventListener('resize', resizeScopes);
resizeScopes();

// Expects each history value pre-normalized to roughly [-1, 1] (or [0, 1]
// for a bottom-anchored trace like accel magnitude).
export function drawTrace(scope) {
  const { canvas, ctx, history, baselineFrac, color } = scope;
  const w = canvas.width, h = canvas.height;
  const baseline = h * baselineFrac;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = 'rgba(0,0,0,0.1)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, baseline);
  ctx.lineTo(w, baseline);
  ctx.stroke();

  if (history.length < 2) return;

  ctx.strokeStyle = color;
  ctx.lineWidth = 2 * devicePixelRatio;
  ctx.beginPath();
  history.forEach((v, i) => {
    const x = (i / (MAX_HISTORY - 1)) * w;
    const y = baseline - v * baseline * 0.9;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

export function pushHistory(history, value) {
  history.push(value);
  if (history.length > MAX_HISTORY) history.shift();
}
