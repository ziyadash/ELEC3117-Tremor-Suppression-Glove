// ============================================================
// Tremor glove telemetry visualizer — ESP32-C5-DevKitC-1
// Reads a 55-byte BLE packet (quaternion + linear accel + gyro + on-device
// tremor status) from tremor_glove/tremor_glove.ino and renders live
// orientation, acceleration, dead-reckoned position, and tremor state.
//
// Split into modules alongside this file: scene.js (Three.js proxy
// board), scopes.js (rolling trace canvases), euler.js (quaternion
// math), position.js (dead-reckoning), tremor.js (tremor status display,
// owns its own toggle button — detection itself runs on-device now).
// This file is the entry point - it wires them together around the Web
// Bluetooth connection.
// ============================================================
import { boardGroup, accelDir, accelArrow, pushTrailPoint, resetTrail } from './scene.js';
import { rollScope, accelScope, drawTrace, pushHistory } from './scopes.js';
import { quatToEuler } from './euler.js';
import { lowPassAccel, integratePosition, resetPositionState, position } from './position.js';
import { updateTremorDisplay, resetTremorDisplay } from './tremor.js';
import { startSession, recordTremorSample, recordMotorSample } from './session.js';

// ---------------- Config: must match the ESP32 firmware UUIDs ----------------
const SERVICE_UUID = 'a1b2c3d4-0001-4a5b-8c6d-1234567890ab';
const CHARACTERISTIC_UUID = 'a1b2c3d4-0002-4a5b-8c6d-1234567890ab';

const connectBtn = document.getElementById('connectBtn');
const statusDot = document.getElementById('statusDot');
const emptyHint = document.getElementById('emptyHint');
const errorBanner = document.getElementById('errorBanner');
const linkStateEl = document.getElementById('linkState');
const pktRateEl = document.getElementById('pktRate');

let packetCount = 0;
setInterval(() => {
  pktRateEl.textContent = packetCount;
  packetCount = 0;
}, 1000);

function showError(msg) {
  errorBanner.textContent = msg;
  errorBanner.style.display = 'block';
  setTimeout(() => { errorBanner.style.display = 'none'; }, 5000);
}

// Accel magnitude scope full-scale: values above this clip visually at the
// top of the trace. Hand tremor is typically well under 1g once gravity is
// removed; 2g leaves headroom for sharper movements.
const ACCEL_SCOPE_FULL_SCALE_G = 2;
const ACCEL_ARROW_MIN_LEN = 0.15;
const ACCEL_ARROW_MAX_LEN = 1.8;

let lastPacketTime = null;

// ============================================================
// BLE notification handling
// ============================================================
function handleNotification(event) {
  const dv = event.target.value;
  if (dv.byteLength < 55) return; // 10 float32s + tremor status fields

  const w = dv.getFloat32(0, true);
  const x = dv.getFloat32(4, true);
  const y = dv.getFloat32(8, true);
  const z = dv.getFloat32(12, true);
  const rawAx = dv.getFloat32(16, true);
  const rawAy = dv.getFloat32(20, true);
  const rawAz = dv.getFloat32(24, true);
  const gx = dv.getFloat32(28, true);
  const gy = dv.getFloat32(32, true);
  const gz = dv.getFloat32(36, true);
  const { x: ax, y: ay, z: az } = lowPassAccel(rawAx, rawAy, rawAz);

  const tremorActive = dv.getUint8(40) !== 0;
  const leadChannel = dv.getUint8(41);
  const tremorRms = dv.getFloat32(42, true);
  const tremorThreshold = dv.getFloat32(46, true);
  const tremorFreqHz = dv.getFloat32(50, true);
  const driveIntensity = dv.getUint8(54);

  packetCount++;

  const now = performance.now();
  const dt = lastPacketTime === null ? 0.02 : Math.min((now - lastPacketTime) / 1000, 0.1);
  lastPacketTime = now;

  document.getElementById('qw').textContent = w.toFixed(4);
  document.getElementById('qx').textContent = x.toFixed(4);
  document.getElementById('qy').textContent = y.toFixed(4);
  document.getElementById('qz').textContent = z.toFixed(4);

  // Apply orientation directly to the 3D board proxy.
  // Note: MPU6050 DMP quaternion order is (w, x, y, z); Three.js Quaternion is (x, y, z, w).
  boardGroup.quaternion.set(x, y, z, w);

  const euler = quatToEuler(w, x, y, z);
  document.getElementById('eRoll').textContent = euler.roll.toFixed(1);
  document.getElementById('ePitch').textContent = euler.pitch.toFixed(1);
  document.getElementById('eYaw').textContent = euler.yaw.toFixed(1);

  const accelMag = Math.sqrt(ax * ax + ay * ay + az * az);
  document.getElementById('aX').textContent = ax.toFixed(3);
  document.getElementById('aY').textContent = ay.toFixed(3);
  document.getElementById('aZ').textContent = az.toFixed(3);
  document.getElementById('aMag').textContent = accelMag.toFixed(3);

  document.getElementById('gX').textContent = gx.toFixed(3);
  document.getElementById('gY').textContent = gy.toFixed(3);
  document.getElementById('gZ').textContent = gz.toFixed(3);

  // Point the arrow along the acceleration vector (body frame - the
  // boardGroup parent rotation composes it into world space automatically).
  if (accelMag > 1e-4) {
    accelDir.set(ax, ay, az).normalize();
    const len = THREE.MathUtils.clamp(accelMag * 1.2, ACCEL_ARROW_MIN_LEN, ACCEL_ARROW_MAX_LEN);
    accelArrow.setDirection(accelDir);
    accelArrow.setLength(len, len * 0.25, len * 0.15);
  }

  pushHistory(rollScope.history, euler.roll / 180);
  pushHistory(accelScope.history, THREE.MathUtils.clamp(accelMag / ACCEL_SCOPE_FULL_SCALE_G, 0, 1));
  drawTrace(rollScope);
  drawTrace(accelScope);

  // Rotate body-frame accel into world frame using the orientation just
  // applied above, then integrate accel -> velocity -> position.
  integratePosition(ax, ay, az, boardGroup.quaternion, dt);
  boardGroup.position.copy(position);
  pushTrailPoint(position);

  document.getElementById('pX').textContent = position.x.toFixed(3);
  document.getElementById('pY').textContent = position.y.toFixed(3);
  document.getElementById('pZ').textContent = position.z.toFixed(3);

  // Tremor detection runs on-device now (tremor_glove/tremor_detect.ino) —
  // this just displays whatever the firmware already decided.
  updateTremorDisplay(tremorActive, leadChannel, tremorRms, tremorThreshold, tremorFreqHz);

  // Recorded regardless of whether the tremor panel is currently shown -
  // that toggle only controls the display, not what the firmware decided.
  // threshold <= 0 means the firmware is still calibrating (see tremor.js) -
  // ratio 0 in that case, same as idle, rather than dividing by zero.
  const tremorRatio = tremorThreshold > 0 ? tremorRms / tremorThreshold : 0;
  recordTremorSample(tremorActive, tremorRatio);
  recordMotorSample(driveIntensity);
}

async function connect() {
  if (!navigator.bluetooth) {
    showError('Web Bluetooth is not available in this browser. Use desktop Chrome/Edge, or Chrome on Android.');
    return;
  }

  try {
    linkStateEl.textContent = 'scanning...';
    const device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [SERVICE_UUID] }]
    });

    linkStateEl.textContent = 'connecting...';
    device.addEventListener('gattserverdisconnected', onDisconnected);

    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE_UUID);
    const characteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);

    await characteristic.startNotifications();
    characteristic.addEventListener('characteristicvaluechanged', handleNotification);

    linkStateEl.textContent = 'live';
    statusDot.classList.add('live');
    emptyHint.style.display = 'none';
    connectBtn.textContent = 'Connected';
    connectBtn.disabled = true;
    startSession();
  } catch (err) {
    console.error(err);
    showError('Connection failed: ' + err.message);
    linkStateEl.textContent = 'idle';
  }
}

function onDisconnected() {
  statusDot.classList.remove('live');
  linkStateEl.textContent = 'disconnected';
  connectBtn.textContent = 'Connect device';
  connectBtn.disabled = false;
  emptyHint.style.display = 'block';

  resetPositionState();
  lastPacketTime = null;
  boardGroup.position.set(0, 0, 0);
  resetTrail();
  resetTremorDisplay();
}

connectBtn.addEventListener('click', connect);
