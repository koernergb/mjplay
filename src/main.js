import loadMujoco from '@mujoco/mujoco';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import ur5ePolicyManifest from './policies/ur5e-wave/policy.json';
import ur5ePolicyUrl from './policies/ur5e-wave/policy.onnx?url';
import { OnnxPolicy, applyJointPositionAction, resolvePolicyActuators } from './policy-runtime.js';

const modelFiles = import.meta.glob('./models/**/*', {
  eager: true,
  query: '?url',
  import: 'default',
});

const MODELS = {
  panda: { label: 'Franka Emika Panda', directory: 'panda', scene: 'scene.xml' },
  ur5e: {
    label: 'Universal Robots UR5e', directory: 'ur5e', scene: 'scene.xml',
    policy: { manifest: ur5ePolicyManifest, url: ur5ePolicyUrl },
  },
};

const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };
const JOINT = { FREE: 0, BALL: 1, SLIDE: 2, HINGE: 3 };
const OBJ = { BODY: 1, JOINT: 3, GEOM: 5, ACTUATOR: 19 };
const $ = (id) => document.getElementById(id);

async function mountModel(mujoco, modelConfig) {
  const marker = `/models/${modelConfig.directory}/`;
  const virtualRoot = `/models/${modelConfig.directory}`;
  mujoco.FS.mkdirTree(`${virtualRoot}/assets`);
  const files = Object.entries(modelFiles).filter(([sourcePath]) => sourcePath.includes(marker));
  await Promise.all(files.map(async ([sourcePath, url]) => {
    if (sourcePath.endsWith('NOTICE.md')) return;
    const bytes = new Uint8Array(await (await fetch(url)).arrayBuffer());
    mujoco.FS.writeFile(`${virtualRoot}/${sourcePath.split(marker)[1]}`, bytes);
  }));
  return `${virtualRoot}/${modelConfig.scene}`;
}

function modelName(mujoco, model, type, id, fallback) {
  try { return mujoco.mj_id2name(model, type, id) || fallback; }
  catch { return fallback; }
}

function meshGeometry(model, meshId) {
  const geometry = new THREE.BufferGeometry();
  const vertexStart = model.mesh_vertadr[meshId];
  const vertexCount = model.mesh_vertnum[meshId];
  const faceStart = model.mesh_faceadr[meshId];
  const faceCount = model.mesh_facenum[meshId];
  const positions = new Float32Array(vertexCount * 3);
  const indices = new Uint32Array(faceCount * 3);

  for (let i = 0; i < positions.length; i++) positions[i] = model.mesh_vert[vertexStart * 3 + i];
  for (let i = 0; i < indices.length; i++) indices[i] = model.mesh_face[faceStart * 3 + i];
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function primitiveGeometry(type, size) {
  const [x, y, z] = size;
  if (type === GEOM.SPHERE) return new THREE.SphereGeometry(x, 24, 16);
  if (type === GEOM.BOX) return new THREE.BoxGeometry(x * 2, y * 2, z * 2);
  if (type === GEOM.CYLINDER) return new THREE.CylinderGeometry(x, x, z * 2, 24);
  if (type === GEOM.CAPSULE) return new THREE.CapsuleGeometry(x, y * 2, 8, 16);
  if (type === GEOM.ELLIPSOID) {
    const geometry = new THREE.SphereGeometry(1, 24, 16);
    geometry.scale(x, y, z);
    return geometry;
  }
  return null;
}

async function main() {
  const selectedKey = new URLSearchParams(location.search).get('model') || 'panda';
  const selectedModel = MODELS[selectedKey] || MODELS.panda;
  $('modelPicker').value = Object.hasOwn(MODELS, selectedKey) ? selectedKey : 'panda';
  $('modelPicker').addEventListener('change', (event) => {
    const url = new URL(location.href);
    url.searchParams.set('model', event.target.value);
    location.assign(url);
  });
  $('status').textContent = 'Loading MuJoCo…';
  const mujoco = await loadMujoco();
  $('status').textContent = `Mounting ${selectedModel.label} assets…`;
  const scenePath = await mountModel(mujoco, selectedModel);
  const model = mujoco.MjModel.from_xml_path(scenePath);
  const data = new mujoco.MjData(model);

  const app = $('app');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(app.clientWidth, app.clientHeight);
  renderer.shadowMap.enabled = true;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  app.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x090d16);
  scene.fog = new THREE.Fog(0x090d16, 4, 9);
  const camera = new THREE.PerspectiveCamera(42, app.clientWidth / app.clientHeight, 0.01, 100);
  camera.up.set(0, 0, 1);
  camera.position.set(2.1, -2.2, 1.45);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.55);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xc9ddff, 0x172033, 1.5));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
  keyLight.position.set(2, -2, 4);
  keyLight.castShadow = true;
  scene.add(keyLight);

  const geomEntries = Array(model.ngeom).fill(null);
  const bodyMeshes = new Map();
  const geometryCache = new Map();
  const redMaterial = new THREE.MeshStandardMaterial({ color: 0xff334f, emissive: 0x7a0617, roughness: 0.38 });

  for (let geomId = 0; geomId < model.ngeom; geomId++) {
    const type = model.geom_type[geomId];
    if (type === GEOM.PLANE) {
      const grid = new THREE.GridHelper(8, 32, 0x46658f, 0x202b3e);
      grid.rotation.x = Math.PI / 2;
      scene.add(grid);
      continue;
    }

    // Menagerie group 3 contains collision hulls that overlap the detailed visual meshes.
    if (model.geom_group[geomId] === 3) continue;
    let geometry;
    if (type === GEOM.MESH) {
      const meshId = model.geom_dataid[geomId];
      if (!geometryCache.has(meshId)) geometryCache.set(meshId, meshGeometry(model, meshId));
      geometry = geometryCache.get(meshId);
    } else {
      geometry = primitiveGeometry(type, [
        model.geom_size[geomId * 3], model.geom_size[geomId * 3 + 1], model.geom_size[geomId * 3 + 2],
      ]);
    }
    if (!geometry) continue;

    const rgba = Array.from(model.geom_rgba.slice(geomId * 4, geomId * 4 + 4));
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      opacity: rgba[3],
      transparent: rgba[3] < 1,
      roughness: 0.55,
      metalness: 0.04,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.matrixAutoUpdate = false;
    scene.add(mesh);

    const bodyId = model.geom_bodyid[geomId];
    const entry = { mesh, material, bodyId, axisFix: type === GEOM.CAPSULE || type === GEOM.CYLINDER };
    geomEntries[geomId] = entry;
    if (!bodyMeshes.has(bodyId)) bodyMeshes.set(bodyId, []);
    bodyMeshes.get(bodyId).push(entry);
  }

  const sliders = [];
  for (let jointId = 0; jointId < model.njnt; jointId++) {
    const type = model.jnt_type[jointId];
    if (type !== JOINT.HINGE && type !== JOINT.SLIDE) continue;
    const min = model.jnt_range[jointId * 2];
    const max = model.jnt_range[jointId * 2 + 1];
    const qposAddress = model.jnt_qposadr[jointId];
    const name = modelName(mujoco, model, OBJ.JOINT, jointId, `joint ${jointId + 1}`);
    const row = document.createElement('label');
    row.className = 'joint';
    row.innerHTML = `<span>${name}</span><output></output><input type="range" min="${min}" max="${max}" step="${type === JOINT.SLIDE ? 0.001 : 0.01}">`;
    const input = row.querySelector('input');
    const output = row.querySelector('output');
    input.addEventListener('input', () => {
      data.qpos[qposAddress] = Number(input.value);
      output.value = Number(input.value).toFixed(type === JOINT.SLIDE ? 3 : 2);
      mujoco.mj_forward(model, data);
    });
    $('joints').appendChild(row);
    sliders.push({ input, output, qposAddress, type });
  }

  function refreshSliders() {
    for (const slider of sliders) {
      slider.input.value = data.qpos[slider.qposAddress];
      slider.output.value = Number(slider.input.value).toFixed(slider.type === JOINT.SLIDE ? 3 : 2);
    }
  }

  let mode = 'pose';
  let policy = null;
  let policyLoading = null;
  let policyPlaying = false;
  let policyPending = false;
  let policyPhase = 0;
  let policyAccumulator = 0;
  let currentControl = Float64Array.from(data.ctrl);
  let policyActuatorIds = [];
  let policyHomeControl = [];
  let policyControlRanges = [];
  let pushRemaining = 0;
  const pushBodyId = selectedModel.policy
    ? mujoco.mj_name2id(model, OBJ.BODY, 'wrist_3_link')
    : -1;

  function reset() {
    if (model.nkey > 0) mujoco.mj_resetDataKeyframe(model, data, 0);
    else mujoco.mj_resetData(model, data);
    mujoco.mj_forward(model, data);
    policyPhase = 0;
    policyAccumulator = 0;
    pushRemaining = 0;
    currentControl = Float64Array.from(data.ctrl);
    if (policyActuatorIds.length) {
      policyHomeControl = policyActuatorIds.map((id) => data.ctrl[id]);
    }
    refreshSliders();
  }

  async function ensurePolicy() {
    if (!selectedModel.policy) throw new Error('No policy is available for this robot');
    if (policy) return policy;
    if (policyLoading) return policyLoading;
    $('policyRuntime').textContent = 'loading…';
    $('status').textContent = 'Loading ONNX policy…';
    policyLoading = (async () => {
      policyActuatorIds = resolvePolicyActuators(mujoco, model, selectedModel.policy.manifest, OBJ.ACTUATOR);
      policyControlRanges = policyActuatorIds.map((id) => [
        model.actuator_ctrlrange[id * 2], model.actuator_ctrlrange[id * 2 + 1],
      ]);
      reset();
      policy = await OnnxPolicy.create(
        selectedModel.policy.manifest,
        selectedModel.policy.url,
        selectedKey,
      );
      $('policyRuntime').textContent = `${policy.executionProvider} · cold ${policy.coldStartMs.toFixed(0)}ms`;
      $('inferenceTime').textContent = `${policy.warmInferenceMs.toFixed(2)}ms warm`;
      $('status').textContent = `${selectedModel.label} policy ready`;
      return policy;
    })().catch((error) => {
      policyLoading = null;
      $('policyRuntime').textContent = 'load failed';
      throw error;
    });
    return policyLoading;
  }

  async function inferPolicy() {
    if (policyPending) return;
    policyPending = true;
    try {
      const runner = await ensurePolicy();
      const amplitude = Number($('policyAmplitude').value);
      const speed = Number($('policySpeed').value);
      const observation = new Float32Array([
        Math.sin(policyPhase) * amplitude,
        Math.cos(policyPhase) * amplitude,
        amplitude,
      ]);
      const rawAction = await runner.run(observation);
      const result = applyJointPositionAction(
        rawAction,
        selectedModel.policy.manifest,
        policyHomeControl,
        policyControlRanges,
      );
      currentControl = Float64Array.from(data.ctrl);
      policyActuatorIds.forEach((id, index) => { currentControl[id] = result.applied[index]; });
      policyPhase += 2 * Math.PI * speed / selectedModel.policy.manifest.control_hz;
      $('inferenceTime').textContent = `${runner.lastInferenceMs.toFixed(2)}ms`;
      $('actionNorm').textContent = result.actionNorm.toFixed(3);
      $('clippedActions').textContent = result.clippedCount;
    } catch (error) {
      policyPlaying = false;
      $('policyPlay').textContent = 'Play';
      $('status').textContent = `Policy stopped: ${error.message}`;
      $('status').classList.add('error');
      console.error('policy inference failed:', error);
    } finally {
      policyPending = false;
    }
  }

  function applyPolicyForces(dt) {
    data.ctrl.set(currentControl);
    data.xfrc_applied.fill(0);
    if (pushRemaining > 0 && pushBodyId > 0) {
      data.xfrc_applied[pushBodyId * 6 + 1] = 65;
      pushRemaining -= dt;
    }
  }

  async function setMode(nextMode) {
    if (nextMode === 'policy') {
      try { await ensurePolicy(); }
      catch (error) {
        $('status').textContent = `Policy failed: ${error.message}`;
        $('status').classList.add('error');
        return;
      }
    }
    mode = nextMode;
    $('poseMode').classList.toggle('active', mode === 'pose');
    $('simMode').classList.toggle('active', mode === 'sim');
    $('policyMode').classList.toggle('active', mode === 'policy');
    $('policyPanel').hidden = mode !== 'policy';
    $('modeText').textContent = mode === 'pose'
      ? 'Pose mode · drag joints directly'
      : mode === 'sim' ? 'Simulation running' : 'ONNX policy control';
    for (const { input } of sliders) input.disabled = mode !== 'pose';
  }
  $('policyMode').disabled = !selectedModel.policy;
  $('poseMode').addEventListener('click', () => { void setMode('pose'); });
  $('simMode').addEventListener('click', () => { void setMode('sim'); });
  $('policyMode').addEventListener('click', () => { void setMode('policy'); });
  $('reset').addEventListener('click', () => {
    policyPlaying = false;
    $('policyPlay').textContent = 'Play';
    reset();
  });
  $('policyPlay').addEventListener('click', async () => {
    await setMode('policy');
    if (mode !== 'policy') return;
    policyPlaying = !policyPlaying;
    $('policyPlay').textContent = policyPlaying ? 'Pause' : 'Play';
  });
  $('policyStep').addEventListener('click', async () => {
    await setMode('policy');
    if (mode !== 'policy') return;
    policyPlaying = false;
    $('policyPlay').textContent = 'Play';
    await inferPolicy();
    const steps = Math.max(1, Math.round((1 / selectedModel.policy.manifest.control_hz) / model.opt.timestep));
    for (let i = 0; i < steps; i++) {
      applyPolicyForces(model.opt.timestep);
      mujoco.mj_step(model, data);
    }
  });
  $('policyPush').addEventListener('click', () => { pushRemaining = 0.18; });
  $('policyAmplitude').addEventListener('input', () => {
    $('amplitudeValue').textContent = Number($('policyAmplitude').value).toFixed(2);
  });
  $('policySpeed').addEventListener('input', () => {
    $('speedValue').textContent = `${Number($('policySpeed').value).toFixed(2)} Hz`;
  });

  const transform = new THREE.Matrix4();
  const axisFix = new THREE.Matrix4().makeRotationX(Math.PI / 2);
  function syncTransforms() {
    for (let geomId = 0; geomId < model.ngeom; geomId++) {
      const entry = geomEntries[geomId];
      if (!entry) continue;
      const p = geomId * 3;
      const r = geomId * 9;
      transform.set(
        data.geom_xmat[r], data.geom_xmat[r + 1], data.geom_xmat[r + 2], data.geom_xpos[p],
        data.geom_xmat[r + 3], data.geom_xmat[r + 4], data.geom_xmat[r + 5], data.geom_xpos[p + 1],
        data.geom_xmat[r + 6], data.geom_xmat[r + 7], data.geom_xmat[r + 8], data.geom_xpos[p + 2],
        0, 0, 0, 1,
      );
      entry.mesh.matrix.copy(transform);
      if (entry.axisFix) entry.mesh.matrix.multiply(axisFix);
    }
  }

  const highlighted = new Set();
  function updateContacts() {
    const activeBodies = new Set();
    const labels = new Set();
    const contacts = data.ncon > 0 ? data.contact : null;
    for (let i = 0; i < data.ncon; i++) {
      const contact = contacts.get(i);
      const ids = [contact.geom1, contact.geom2];
      ids.forEach((geomId) => activeBodies.add(model.geom_bodyid[geomId]));
      labels.add(ids.map((geomId) => modelName(mujoco, model, OBJ.GEOM, geomId, `geom ${geomId}`)).join(' ↔ '));
      contact.delete();
    }
    contacts?.delete();

    for (const entry of highlighted) {
      if (!activeBodies.has(entry.bodyId)) {
        entry.mesh.material = entry.material;
        highlighted.delete(entry);
      }
    }
    for (const bodyId of activeBodies) {
      for (const entry of bodyMeshes.get(bodyId) || []) {
        entry.mesh.material = redMaterial;
        highlighted.add(entry);
      }
    }
    $('ncon').textContent = data.ncon;
    $('ncon').classList.toggle('hot', data.ncon > 0);
    $('contacts').textContent = labels.size ? [...labels].slice(0, 3).join('\n') : 'No active contacts';
  }

  reset();
  void setMode('pose');
  $('modelStats').textContent = `${model.nbody} bodies · ${model.njnt} joints · ${model.ngeom} geoms`;
  $('status').textContent = `${selectedModel.label} ready`;

  let accumulator = 0;
  let previous = performance.now();
  function frame(now) {
    requestAnimationFrame(frame);
    const elapsed = Math.min((now - previous) / 1000, 0.05);
    previous = now;
    if (mode === 'sim' || (mode === 'policy' && policyPlaying)) {
      accumulator += elapsed;
      while (accumulator >= model.opt.timestep) {
        if (mode === 'policy') {
          policyAccumulator += model.opt.timestep;
          const policyDt = 1 / selectedModel.policy.manifest.control_hz;
          if (policyAccumulator >= policyDt) {
            policyAccumulator %= policyDt;
            void inferPolicy();
          }
          applyPolicyForces(model.opt.timestep);
        }
        mujoco.mj_step(model, data);
        accumulator -= model.opt.timestep;
      }
      refreshSliders();
    } else {
      accumulator = 0;
      mujoco.mj_forward(model, data);
    }
    updateContacts();
    syncTransforms();
    $('time').textContent = `${data.time.toFixed(2)}s`;
    controls.update();
    renderer.render(scene, camera);
  }
  requestAnimationFrame(frame);

  addEventListener('resize', () => {
    camera.aspect = app.clientWidth / app.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(app.clientWidth, app.clientHeight);
  });
}

main().catch((error) => {
  console.error('mjplay failed to start:', error);
  $('status').textContent = `Startup failed: ${error?.message || error}`;
  $('status').classList.add('error');
});
