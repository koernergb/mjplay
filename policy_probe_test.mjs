import fs from 'node:fs/promises';
import { OnnxPolicy, applyJointPositionAction, validatePolicyManifest } from './src/policy-runtime.js';

const manifest = JSON.parse(await fs.readFile('src/policies/ur5e-wave/policy.json', 'utf8'));
const modelBytes = await fs.readFile('src/policies/ur5e-wave/policy.onnx');
validatePolicyManifest(manifest, 'ur5e');

const policy = await OnnxPolicy.create(manifest, modelBytes, 'ur5e');
const action = await policy.run(new Float32Array([0.5, -0.25, 0.75]));
const expected = [0.0125, -0.275, 0.3125, -0.025, -0.2125, 0.3125];
for (let i = 0; i < expected.length; i++) {
  if (Math.abs(action[i] - expected[i]) > 1e-6) {
    throw new Error(`Policy output ${i} was ${action[i]}, expected ${expected[i]}`);
  }
}

const applied = applyJointPositionAction(
  action,
  manifest,
  [-1.5, -1.5, 1.5, -1.5, -1.5, 0],
  Array.from({ length: 6 }, () => [-3.14, 3.14]),
);
if (applied.applied.length !== 6 || applied.clippedCount !== 0) {
  throw new Error('Action adapter produced an invalid result');
}

console.log(`[Policy] provider=${policy.executionProvider}, cold=${policy.coldStartMs.toFixed(2)}ms, warm=${policy.warmInferenceMs.toFixed(2)}ms`);
console.log(`[Policy] output=${Array.from(action, (value) => value.toFixed(4)).join(', ')}`);
console.log('[Policy] ✓ manifest validation, ONNX inference, and action adaptation passed.');
