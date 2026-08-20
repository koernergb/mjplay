const SUPPORTED_SCHEMA_VERSION = 1;

export function validatePolicyManifest(manifest, expectedRobot) {
  const fail = (field, message) => { throw new Error(`policy.json ${field}: ${message}`); };
  if (!manifest || typeof manifest !== 'object') fail('', 'must contain an object');
  if (manifest.schema_version !== SUPPORTED_SCHEMA_VERSION) fail('schema_version', `expected ${SUPPORTED_SCHEMA_VERSION}`);
  if (manifest.robot !== expectedRobot) fail('robot', `expected "${expectedRobot}"`);
  if (manifest.runtime !== 'onnx') fail('runtime', 'only "onnx" is supported');
  if (!Number.isFinite(manifest.control_hz) || manifest.control_hz < 1 || manifest.control_hz > 240) {
    fail('control_hz', 'must be between 1 and 240');
  }
  if (!Number.isInteger(manifest.observation?.size) || manifest.observation.size < 1) {
    fail('observation.size', 'must be a positive integer');
  }
  if (manifest.observation.terms?.length !== manifest.observation.size) {
    fail('observation.terms', 'length must equal observation.size');
  }
  if (!Number.isInteger(manifest.action?.size) || manifest.action.size < 1) {
    fail('action.size', 'must be a positive integer');
  }
  if (manifest.action.type !== 'joint_position_target') {
    fail('action.type', 'only "joint_position_target" is supported');
  }
  if (manifest.action.actuators?.length !== manifest.action.size) {
    fail('action.actuators', 'length must equal action.size');
  }
  if (!Number.isFinite(manifest.action.scale) || manifest.action.scale <= 0) {
    fail('action.scale', 'must be positive');
  }
  if (!Number.isFinite(manifest.action.clip) || manifest.action.clip <= 0) {
    fail('action.clip', 'must be positive');
  }
  return manifest;
}

export function resolvePolicyActuators(mujoco, model, manifest, actuatorObjectType = 19) {
  return manifest.action.actuators.map((name) => {
    const id = mujoco.mj_name2id(model, actuatorObjectType, name);
    if (id < 0) throw new Error(`policy actuator "${name}" does not exist in the MuJoCo model`);
    return id;
  });
}

export class OnnxPolicy {
  constructor(manifest, session, ort, executionProvider, coldStartMs) {
    this.manifest = manifest;
    this.session = session;
    this.ort = ort;
    this.executionProvider = executionProvider;
    this.coldStartMs = coldStartMs;
    this.lastInferenceMs = 0;
    this.warmInferenceMs = 0;
  }

  static async create(manifest, modelUrl, expectedRobot) {
    validatePolicyManifest(manifest, expectedRobot);
    const started = performance.now();
    const ort = await import('onnxruntime-web/wasm');
    ort.env.wasm.numThreads = 1;
    const session = await ort.InferenceSession.create(modelUrl, { executionProviders: ['wasm'] });
    const policy = new OnnxPolicy(manifest, session, ort, 'wasm', performance.now() - started);
    const zero = new Float32Array(manifest.observation.size);
    await policy.run(zero);
    policy.warmInferenceMs = policy.lastInferenceMs;
    return policy;
  }

  async run(observation) {
    if (observation.length !== this.manifest.observation.size) {
      throw new Error(`policy observation has ${observation.length} values; expected ${this.manifest.observation.size}`);
    }
    for (const value of observation) {
      if (!Number.isFinite(value)) throw new Error('policy observation contains a non-finite value');
    }
    const tensor = new this.ort.Tensor('float32', Float32Array.from(observation), [1, observation.length]);
    const started = performance.now();
    const outputs = await this.session.run({ [this.manifest.observation.input_name]: tensor });
    this.lastInferenceMs = performance.now() - started;
    const action = outputs[this.manifest.action.output_name]?.data;
    if (!action || action.length !== this.manifest.action.size) {
      throw new Error(`policy action has ${action?.length ?? 0} values; expected ${this.manifest.action.size}`);
    }
    return Float32Array.from(action);
  }
}

export function applyJointPositionAction(rawAction, manifest, homeControl, controlRanges) {
  const applied = new Float64Array(rawAction.length);
  let normSquared = 0;
  let clippedCount = 0;
  for (let i = 0; i < rawAction.length; i++) {
    if (!Number.isFinite(rawAction[i])) throw new Error('policy action contains a non-finite value');
    const clipped = Math.max(-manifest.action.clip, Math.min(manifest.action.clip, rawAction[i]));
    if (clipped !== rawAction[i]) clippedCount += 1;
    const target = homeControl[i] + manifest.action.scale * clipped;
    applied[i] = Math.max(controlRanges[i][0], Math.min(controlRanges[i][1], target));
    normSquared += rawAction[i] * rawAction[i];
  }
  return { applied, actionNorm: Math.sqrt(normSquared), clippedCount };
}
