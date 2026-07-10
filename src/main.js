import loadMujoco from '@mujoco/mujoco';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { SCENE_XML } from './scene.js';

// ---- MuJoCo geom type enum (matches mjtGeom ordering) ----
const GEOM = { PLANE: 0, HFIELD: 1, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4,
              CYLINDER: 5, BOX: 6, MESH: 7 };

const $ = (id) => document.getElementById(id);

async function main() {
  // 1) Load the official WASM module.
  const mujoco = await loadMujoco();

  // 2) Load the model from an XML string (no VFS / asset files needed for M0).
  const model = mujoco.MjModel.from_xml_string(SCENE_XML);
  const data  = new mujoco.MjData(model);

  // 3) Report structural counts (M0.2 DONE check).
  $('nbody').textContent = model.nbody;
  $('njnt').textContent  = model.njnt;
  $('ngeom').textContent = model.ngeom;
  $('nqnu').textContent  = `${model.nq} / ${model.nu}`;
  console.log('[M0.2] nbody=%d njnt=%d ngeom=%d nq=%d nu=%d',
    model.nbody, model.njnt, model.ngeom, model.nq, model.nu);

  // ---- Three.js setup ----
  const app = $('app');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(app.clientWidth, app.clientHeight);
  renderer.shadowMap.enabled = true;
  app.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1115);

  const camera = new THREE.PerspectiveCamera(
    45, app.clientWidth / app.clientHeight, 0.01, 100);
  camera.position.set(1.6, 1.2, 1.8);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0.3, 0.3, 0);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x1a1d24, 0.9));
  const dir = new THREE.DirectionalLight(0xffffff, 1.1);
  dir.position.set(2, 4, 3); dir.castShadow = true; scene.add(dir);

  // 4) Build one Three mesh per MuJoCo geom (M0.3).
  //    Read flat typed-array views for geom properties.
  const geom_type = model.geom_type;
  const geom_size = model.geom_size;   // 3 per geom
  const geom_rgba = model.geom_rgba;   // 4 per geom
  const geomMeshes = [];               // index -> { mesh, baseMat }  (null for planes)

  for (let g = 0; g < model.ngeom; g++) {
    const t = geom_type[g];
    const sx = geom_size[g*3+0], sy = geom_size[g*3+1], sz = geom_size[g*3+2];
    const r = geom_rgba[g*4+0], gg = geom_rgba[g*4+1],
          b = geom_rgba[g*4+2], a = geom_rgba[g*4+3];

    let geo = null;
    if (t === GEOM.SPHERE)        geo = new THREE.SphereGeometry(sx, 24, 16);
    else if (t === GEOM.BOX)      geo = new THREE.BoxGeometry(sx*2, sy*2, sz*2);
    else if (t === GEOM.CYLINDER) geo = new THREE.CylinderGeometry(sx, sx, sz*2, 24);
    else if (t === GEOM.CAPSULE)  geo = new THREE.CapsuleGeometry(sx, sy*2, 8, 16);
    else if (t === GEOM.ELLIPSOID){ geo = new THREE.SphereGeometry(1,24,16);
                                    geo.scale(sx, sy, sz); }
    else if (t === GEOM.PLANE) {
      // Render the floor as a large flat grid; not tracked for contact colour.
      const grid = new THREE.GridHelper(6, 24, 0x2a2f3a, 0x20242e);
      grid.rotation.x = Math.PI / 2;   // MuJoCo plane spans local x-y
      scene.add(grid);
      geomMeshes.push(null);
      continue;
    }

    if (!geo) { geomMeshes.push(null); continue; } // unsupported (e.g. mesh) in M0

    const baseMat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(r, gg, b), transparent: a < 1, opacity: a,
      roughness: 0.6, metalness: 0.05 });
    const mesh = new THREE.Mesh(geo, baseMat);
    mesh.castShadow = true; mesh.receiveShadow = true;

    // MuJoCo capsule/cylinder long-axis is local Z; Three's is local Y. Rotate.
    if (t === GEOM.CAPSULE || t === GEOM.CYLINDER) mesh.rotation.x = Math.PI / 2;

    scene.add(mesh);
    geomMeshes.push({ mesh, baseMat });
  }

  const redMat = new THREE.MeshStandardMaterial({
    color: 0xff4444, emissive: 0x551111, roughness: 0.4 });

  // 5) Sync Three mesh transforms from MuJoCo geom world pose.
  //    geom_xpos (3/geom) and geom_xmat (9/geom, row-major) are live views.
  const m4 = new THREE.Matrix4();
  function syncTransforms() {
    const xpos = data.geom_xpos, xmat = data.geom_xmat;
    for (let g = 0; g < model.ngeom; g++) {
      const entry = geomMeshes[g];
      if (!entry) continue;
      const o = g*3, r = g*9;
      // MuJoCo row-major 3x3 -> Three Matrix4 (column-major set() takes row args).
      m4.set(
        xmat[r+0], xmat[r+1], xmat[r+2], xpos[o+0],
        xmat[r+3], xmat[r+4], xmat[r+5], xpos[o+1],
        xmat[r+6], xmat[r+7], xmat[r+8], xpos[o+2],
        0, 0, 0, 1);
      entry.mesh.matrixAutoUpdate = false;
      entry.mesh.matrix.copy(m4);
      // Re-apply the axis fix for capsules/cylinders on top of the pose.
      if (entry.mesh.geometry.type === 'CapsuleGeometry' ||
          entry.mesh.geometry.type === 'CylinderGeometry') {
        entry.mesh.matrix.multiply(
          new THREE.Matrix4().makeRotationX(Math.PI / 2));
      }
    }
  }

  // 6) THE CONTACT PROBE (M0.4). data.contact is a COPY: re-fetch + delete each step.
  const inContact = new Set();
  function readContacts() {
    inContact.clear();
    const ncon = data.ncon;
    if (ncon > 0) {
      const contacts = data.contact;             // fresh copy this step
      for (let i = 0; i < ncon; i++) {
        const c = contacts.get(i);
        if (!c) continue;
        inContact.add(c.geom1); inContact.add(c.geom2);
        c.delete();                              // free the element handle
      }
      contacts.delete();                         // free the vector handle
    }
    return ncon;
  }

  // 7) Apply red highlight as a diff (only touch changed meshes -> stays 60fps).
  const currentlyRed = new Set();
  function applyHighlight() {
    for (const g of inContact) {
      const e = geomMeshes[g];
      if (e && !currentlyRed.has(g)) e.mesh.material = redMat;
    }
    for (const g of [...currentlyRed]) {
      if (!inContact.has(g)) {
        const e = geomMeshes[g];
        if (e) e.mesh.material = e.baseMat;
        currentlyRed.delete(g);
      }
    }
    for (const g of inContact) currentlyRed.add(g);
  }

  // 8) Reset button (M1.1 preview — resets state to the model's initial pose).
  $('reset').addEventListener('click', () => {
    mujoco.mj_resetData(model, data);
    mujoco.mj_forward(model, data);
  });

  // 9) Fixed-timestep loop.
  const dt = model.opt.timestep;
  let acc = 0, last = performance.now();
  let loggedFirstContact = false;

  function frame(now) {
    requestAnimationFrame(frame);
    acc += Math.min((now - last) / 1000, 0.05); last = now;
    while (acc >= dt) { mujoco.mj_step(model, data); acc -= dt; }

    const ncon = readContacts();
    applyHighlight();
    syncTransforms();

    // HUD
    $('time').textContent = data.time.toFixed(2) + 's';
    const nconEl = $('ncon');
    nconEl.textContent = ncon;
    nconEl.classList.toggle('hot', ncon > 0);

    // M0.4 DONE check: log the first real contact with geom IDs, once.
    if (ncon > 0 && !loggedFirstContact) {
      loggedFirstContact = true;
      const c2 = data.contact;
      console.log('[M0.4] FIRST CONTACT at t=%ss, ncon=%d', data.time.toFixed(3), ncon);
      for (let i = 0; i < ncon; i++) {
        const c = c2.get(i);
        console.log('  contact %d: geom1=%d geom2=%d dist=%f',
          i, c.geom1, c.geom2, c.dist);
        c.delete();
      }
      c2.delete();
      console.log('[M0.4] ✓ GO — contacts are readable from JS.');
    }

    controls.update();
    renderer.render(scene, camera);
  }

  // initial forward so first frame is posed correctly, then run.
  mujoco.mj_forward(model, data);
  requestAnimationFrame(frame);

  addEventListener('resize', () => {
    camera.aspect = app.clientWidth / app.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(app.clientWidth, app.clientHeight);
  });
}

main().catch((e) => {
  console.error('mjplay failed to start:', e);
  document.getElementById('hud').innerHTML =
    '<h1 style="color:#ff5c5c">startup failed</h1>' +
    '<div style="color:#e6e8eb">' + (e?.message || e) + '</div>' +
    '<div class="hint">Check the console. Most likely the .wasm asset isn\\'t being served.</div>';
});
