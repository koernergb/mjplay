import fs from 'node:fs/promises';
import path from 'node:path';
import loadMujoco from '@mujoco/mujoco';
import {
  OnnxPolicy,
  applyJointPositionAction,
  buildGo2Observation,
  jointPositionTargetsToTorque,
  resolvePolicyActuators,
  validatePolicyManifest,
} from './src/policy-runtime.js';

const manifest = JSON.parse(await fs.readFile('src/policies/go2-velocity-flat/policy.json', 'utf8'));
validatePolicyManifest(manifest, 'go2');

const mujoco = await loadMujoco();
const sourceRoot = path.resolve('src/models/go2');
const virtualRoot = '/models/go2';
mujoco.FS.mkdirTree(`${virtualRoot}/assets`);

async function mountDirectory(relative = '') {
  for (const entry of await fs.readdir(path.join(sourceRoot, relative), { withFileTypes: true })) {
    const child = path.join(relative, entry.name);
    if (entry.isDirectory()) await mountDirectory(child);
    else if (!entry.name.endsWith('.md') && !entry.name.endsWith('.png')) {
      mujoco.FS.writeFile(`${virtualRoot}/${child}`, await fs.readFile(path.join(sourceRoot, child)));
    }
  }
}

await mountDirectory();
const model = mujoco.MjModel.from_xml_path(`${virtualRoot}/scene.xml`);
const data = new mujoco.MjData(model);
mujoco.mj_resetDataKeyframe(model, data, 0);
mujoco.mj_forward(model, data);

const actuatorIds = resolvePolicyActuators(mujoco, model, manifest);
const jointIds = manifest.action.joints.map((name) => mujoco.mj_name2id(model, 3, name));
if (jointIds.some((id) => id < 0)) throw new Error('Go2 policy joint mapping is incomplete');
const qposAddresses = jointIds.map((id) => model.jnt_qposadr[id]);
const dofAddresses = jointIds.map((id) => model.jnt_dofadr[id]);
const torqueRanges = actuatorIds.map((id) => [model.actuator_ctrlrange[id * 2], model.actuator_ctrlrange[id * 2 + 1]]);
const jointRanges = jointIds.map((id) => [model.jnt_range[id * 2], model.jnt_range[id * 2 + 1]]);

const policy = await OnnxPolicy.create(
  manifest,
  await fs.readFile('src/policies/go2-velocity-flat/policy.onnx'),
  'go2',
  [{ path: 'policy.onnx.data', data: await fs.readFile('src/policies/go2-velocity-flat/policy.onnx.data') }],
);
let previousAction = new Float32Array(12);
let targets = Float64Array.from(manifest.action.default_pose);
const physicsStepsPerPolicyStep = Math.round((1 / manifest.control_hz) / model.opt.timestep);

for (let policyStep = 0; policyStep < 50; policyStep++) {
  const observation = buildGo2Observation(
    data, qposAddresses, dofAddresses, manifest.action.default_pose, [0.6, 0, 0], previousAction,
  );
  if (observation.length !== 45) throw new Error(`Go2 observation length is ${observation.length}, expected 45`);
  const rawAction = await policy.run(observation);
  previousAction = rawAction;
  targets = applyJointPositionAction(rawAction, manifest, manifest.action.default_pose, jointRanges).applied;

  for (let step = 0; step < physicsStepsPerPolicyStep; step++) {
    const torque = jointPositionTargetsToTorque(
      targets,
      qposAddresses.map((address) => data.qpos[address]),
      dofAddresses.map((address) => data.qvel[address]),
      manifest.controller.stiffness,
      manifest.controller.damping,
      torqueRanges,
    );
    actuatorIds.forEach((id, index) => { data.ctrl[id] = torque[index]; });
    mujoco.mj_step(model, data);
  }
}

if (!Number.isFinite(data.qpos[2]) || data.qpos[2] < 0.15) {
  throw new Error(`Go2 rollout fell or became invalid: base height=${data.qpos[2]}`);
}
console.log(`[Go2 policy] provider=${policy.executionProvider}, input=45, output=12, height=${data.qpos[2].toFixed(3)}m`);
console.log('[Go2 policy] ✓ pinned ONNX weights, observation adapter, PD controller, and 1s rollout passed.');

data.delete();
model.delete();
