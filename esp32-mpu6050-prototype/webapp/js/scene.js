// ============================================================
// Three.js scene
// ============================================================
const holder = document.getElementById('canvas-holder');
const scene = new THREE.Scene();

// Orthographic, not perspective: under perspective, a long/thin rotating
// object (the board, 1.6 x 0.12 x 1.0) can get near/far-end size ratios
// over 1.5x when its long axis points toward the camera, which reads as
// the box tapering to a wedge - easy to mistake for clipping. Orthographic
// projection has no such distance-based scaling, so the box's true shape
// stays legible from every orientation.
const ORTHO_VIEW_HALF_HEIGHT = 3;
const camera = new THREE.OrthographicCamera(
  -ORTHO_VIEW_HALF_HEIGHT, ORTHO_VIEW_HALF_HEIGHT,
  ORTHO_VIEW_HALF_HEIGHT, -ORTHO_VIEW_HALF_HEIGHT,
  0.1, 100
);
camera.position.set(2.4, 1.8, 2.4);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
holder.appendChild(renderer.domElement);

function resizeRenderer() {
  const w = holder.clientWidth, h = holder.clientHeight;
  renderer.setSize(w, h);
  const aspect = w / h;
  camera.left = -ORTHO_VIEW_HALF_HEIGHT * aspect;
  camera.right = ORTHO_VIEW_HALF_HEIGHT * aspect;
  camera.top = ORTHO_VIEW_HALF_HEIGHT;
  camera.bottom = -ORTHO_VIEW_HALF_HEIGHT;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resizeRenderer);
resizeRenderer();

// Subtle grid floor for spatial reference. Offset well below the board's
// max rotated extent (~0.945 at the corners) plus the position dead-band
// below, so a tumbling/drifting board never pokes through the floor plane.
const grid = new THREE.GridHelper(6, 24, 0xc7ccd1, 0xe6e9ec);
grid.position.y = -2.2;
scene.add(grid);

// Board-like proxy object: a flattened box with an axis indicator strip,
// standing in for the MPU6050 breakout board itself.
export const boardGroup = new THREE.Group();

const boardGeo = new THREE.BoxGeometry(1.6, 0.12, 1.0);
const boardMat = new THREE.MeshStandardMaterial({ color: 0x3e4c59, metalness: 0.3, roughness: 0.55 });
const board = new THREE.Mesh(boardGeo, boardMat);
boardGroup.add(board);

// Body-frame axes gizmo (x=red, y=green, z=blue), sized to poke out past
// the board's edges so it stays visible from any orientation.
const bodyAxes = new THREE.AxesHelper(1.0);
boardGroup.add(bodyAxes);

// Let the strip protrude past the board's front edge (board's z extends to
// 0.5) rather than sit flush with it or stop short of it: flush caused the
// two faces to sit exactly coplanar (a z-fighting flicker), and stopping
// short exposed a thin sliver of bare board at the seam. Poking out past
// the edge covers the seam completely with no coincident face.
const stripGeo = new THREE.BoxGeometry(1.6, 0.13, 0.16);
export const stripMat = new THREE.MeshStandardMaterial({ color: 0x3b82f6, emissive: 0x1e3a6b, metalness: 0.2, roughness: 0.4 });
const strip = new THREE.Mesh(stripGeo, stripMat);
strip.position.z = 0.46;
boardGroup.add(strip);

// Accel vector arrow: a child of boardGroup because the DMP's linear-accel
// output is expressed in the sensor's body frame - parenting it here lets
// three.js's transform hierarchy compose that with the board's current
// orientation to get the correct world-space direction for free.
export const accelDir = new THREE.Vector3(0, 1, 0);
export const accelArrow = new THREE.ArrowHelper(accelDir, new THREE.Vector3(0, 0, 0), 0.01, 0xe3a34d, 0.12, 0.08);
boardGroup.add(accelArrow);

scene.add(boardGroup);

// Fading trail of the board's recent estimated position, so translation
// through space reads as a path rather than just a jump.
const TRAIL_LENGTH = 120;
const trailPositions = new Float32Array(TRAIL_LENGTH * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailMat = new THREE.LineBasicMaterial({ color: 0xe3a34d, transparent: true, opacity: 0.45 });
const trailLine = new THREE.Line(trailGeo, trailMat);
scene.add(trailLine);

export function pushTrailPoint(v) {
  trailPositions.copyWithin(0, 3);
  trailPositions[TRAIL_LENGTH * 3 - 3] = v.x;
  trailPositions[TRAIL_LENGTH * 3 - 2] = v.y;
  trailPositions[TRAIL_LENGTH * 3 - 1] = v.z;
  trailGeo.attributes.position.needsUpdate = true;
}

export function resetTrail() {
  trailPositions.fill(0);
  trailGeo.attributes.position.needsUpdate = true;
}

scene.add(new THREE.AmbientLight(0x8899aa, 0.6));
const key = new THREE.DirectionalLight(0xffffff, 0.9);
key.position.set(3, 4, 2);
scene.add(key);

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();
