// ============================================================
// Three.js scene
// ============================================================
const holder = document.getElementById('canvas-holder');
const scene = new THREE.Scene();

// Orthographic, not perspective: under perspective, a long/thin rotating
// object can get near/far-end size ratios over 1.5x when its long axis
// points toward the camera, which reads as tapering to a wedge - easy to
// mistake for clipping. Orthographic projection has no such distance-based
// scaling, so the hand's true shape stays legible from every orientation.
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

// Subtle grid floor for spatial reference. Offset well below the hand's
// max rotated extent (fingertip-to-pivot, scaled, ~1.45) plus the position
// dead-reckoning clamp (1.0) below, so a tumbling/drifting hand never
// pokes through the floor plane in the worst case.
const grid = new THREE.GridHelper(6, 24, 0xc7ccd1, 0xe6e9ec);
grid.position.y = -2.6;
scene.add(grid);

// Hand proxy object, standing in for the glove. Loaded async from an OBJ
// file (Poly Pizza "Low Poly Right Hand" by Raziq Brown, CC BY 3.0 - see
// note.md), since there's no primitive-built fallback. The tremor tint
// (stripMat) is applied directly to the hand mesh itself, so the whole
// hand glows pink while tremor detection is ACTIVE.
export const boardGroup = new THREE.Group();

export const stripMat = new THREE.MeshStandardMaterial({ color: 0x3b82f6, emissive: 0x1e3a6b, metalness: 0.25, roughness: 0.5 });

// Body-frame axes gizmo (x=red, y=green, z=blue), sized to poke out past
// the hand so it stays visible from any orientation.
const bodyAxes = new THREE.AxesHelper(1.0);
boardGroup.add(bodyAxes);

// The source mesh is a fragment cut from a full-body rig: it sits at an
// arbitrary world offset rather than being centered/oriented for standalone
// use. Recenter it on its wrist end (bounding-box max-Y - the model was
// exported hanging at the character's side, wrist up/fingers down) and
// rescale so it reads at roughly the same on-screen size the old board
// proxy did.
const HAND_SCALE = 3.5;
new THREE.OBJLoader().load('assets/models/RightHand.obj', (obj) => {
  const mesh = obj.children[0];
  mesh.geometry.computeBoundingBox();
  const bbox = mesh.geometry.boundingBox;
  const centerX = (bbox.min.x + bbox.max.x) / 2;
  const centerZ = (bbox.min.z + bbox.max.z) / 2;
  mesh.geometry.translate(-centerX, -bbox.max.y, -centerZ);
  mesh.geometry.scale(HAND_SCALE, HAND_SCALE, HAND_SCALE);
  mesh.geometry.computeVertexNormals();
  mesh.material = stripMat;
  boardGroup.add(mesh);
});

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
