// ============================================================
// Position dead-reckoning
// ============================================================
// A MEMS accelerometer alone can't give true absolute position - double-
// integrating its noise and bias drifts by meters within seconds. Instead
// of pretending to be accurate, this bleeds off velocity and re-centers
// position every update (a leaky integrator), so it reads as "the board
// is moving this way right now" rather than a real trajectory.
const G_TO_MS2 = 9.80665;
const VELOCITY_DAMPING = 0.90;
const POSITION_RECENTER = 0.985;
const POSITION_CLAMP = 1.0;

export const velocity = new THREE.Vector3();
export const position = new THREE.Vector3();
const worldAccel = new THREE.Vector3();

// One-pole low-pass on the raw accel signal - knocks down sensor/BLE-level
// jitter before it hits the readouts, the arrow, the scope, or (most
// noise-sensitive of all) the position integration below. Lower alpha =
// smoother but slower to respond; 0.3 keeps most tremor-band motion
// (~3-12Hz) intact while cutting higher-frequency noise. Raise it if the
// glove is meant to visualize fast/sharp motion rather than tremor.
const ACCEL_FILTER_ALPHA = 0.3;
const accelFiltered = new THREE.Vector3();
let accelFilterInitialized = false;

export function lowPassAccel(rawX, rawY, rawZ) {
  if (!accelFilterInitialized) {
    accelFiltered.set(rawX, rawY, rawZ);
    accelFilterInitialized = true;
  } else {
    accelFiltered.x += ACCEL_FILTER_ALPHA * (rawX - accelFiltered.x);
    accelFiltered.y += ACCEL_FILTER_ALPHA * (rawY - accelFiltered.y);
    accelFiltered.z += ACCEL_FILTER_ALPHA * (rawZ - accelFiltered.z);
  }
  return accelFiltered;
}

// Rotate body-frame accel into world frame using the orientation just
// applied to the board proxy, then integrate accel -> velocity -> position.
export function integratePosition(ax, ay, az, quaternion, dt) {
  worldAccel.set(ax, ay, az).applyQuaternion(quaternion).multiplyScalar(G_TO_MS2);
  velocity.addScaledVector(worldAccel, dt).multiplyScalar(VELOCITY_DAMPING);
  position.addScaledVector(velocity, dt).multiplyScalar(POSITION_RECENTER);
  position.clampLength(0, POSITION_CLAMP);
  return position;
}

export function resetPositionState() {
  velocity.set(0, 0, 0);
  position.set(0, 0, 0);
  accelFilterInitialized = false;
}
