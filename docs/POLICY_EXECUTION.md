# Policy Execution in mjplay

## Product direction

mjplay should become an interactive browser lab for running, perturbing, and
debugging robot policies—not merely a MuJoCo renderer.

> Load a robot and a policy, run it interactively, then understand why it
> succeeds or fails.

MuJoCo supplies the physics engine. mjplay supplies the policy packaging,
observation and action adapters, interactive controls, diagnostics, comparison,
and shareable playback experience.

The first completion gate is intentionally narrow:

> A bundled Unitree Go2 locomotion policy runs in the browser, responds to
> forward/turn commands, can be pushed off balance, and exposes its actions,
> contacts, and reward while the user pauses, steps, slows, and resets it.

## User experience

The initial policy lab should provide:

- Play, pause, reset, and single-step controls
- Real-time and slow-motion playback speeds
- Forward, lateral, and yaw command inputs
- A push disturbance control with direction and magnitude
- Live observation, action, contact, and reward summaries
- A short rolling timeline for actions, contacts, and reward components
- A visible policy/simulation frequency indicator
- A clear failure state when the robot falls or the policy contract is invalid

A typical session:

1. Select the bundled Go2 locomotion demo.
2. Press Play and command the robot to walk forward.
3. Change its speed or steering command while it moves.
4. Apply a lateral push.
5. Watch the policy attempt to recover.
6. Pause or single-step near a failure.
7. Inspect the policy actions, contact sequence, and reward terms.
8. Reset to the exact initial state and retry with different commands.

## Architecture

```text
Policy package
  ├── scene.xml + assets
  ├── policy.onnx
  └── policy.json
          │
          ▼
Package loader and validator
          │
          ├──► MuJoCo WASM model/data
          │
          └──► ONNX Runtime Web session
                    │
MuJoCo state ──► Observation adapter
                    │
                    ▼
                ONNX policy
                    │
                    ▼
              Action adapter
                    │
                    ▼
                data.ctrl
                    │
                    ▼
                 mj_step
                    │
                    ├──► Three.js renderer
                    └──► diagnostics/timeline
```

The simulation and policy loops must be separate. MuJoCo may run at 200–500 Hz
while policy inference runs at 25–100 Hz. The most recent action is held between
policy evaluations.

```js
while (accumulator >= physicsDt) {
  if (policyStepsUntilUpdate === 0) {
    const observation = observationAdapter.read(model, data, command, history);
    const rawAction = await policy.run(observation);
    currentControl = actionAdapter.convert(rawAction, model, data);
    policyStepsUntilUpdate = physicsStepsPerPolicyStep;
  }

  data.ctrl.set(currentControl);
  mujoco.mj_step(model, data);
  diagnostics.sample(model, data, currentControl);
  policyStepsUntilUpdate -= 1;
  accumulator -= physicsDt;
}
```

Inference should not be awaited inside a synchronous multi-step loop in the
final implementation. The scheduler should either precompute the next action or
yield at policy boundaries so rendering and input remain responsive.

## Policy package contract

An arbitrary ONNX file is insufficient because observation and action layouts
are training-specific. Every policy must ship with metadata that declares its
runtime contract.

```text
go2-walk/
  scene.xml
  robot.xml
  assets/
  policy.onnx
  policy.json
```

Example `policy.json`:

```json
{
  "schema_version": 1,
  "id": "go2-walk-v1",
  "name": "Go2 walking policy",
  "robot": "unitree_go2",
  "runtime": "onnx",
  "control_hz": 50,
  "physics_hz": 200,
  "observation": {
    "size": 48,
    "terms": [
      { "name": "base_angular_velocity", "scale": 0.25 },
      { "name": "projected_gravity" },
      { "name": "command", "size": 3 },
      { "name": "joint_position_error", "order": ["FL_hip", "FL_thigh", "FL_calf"] },
      { "name": "joint_velocity", "scale": 0.05 },
      { "name": "previous_action" }
    ],
    "clip": 100.0
  },
  "action": {
    "type": "joint_position_target",
    "size": 12,
    "scale": 0.25,
    "clip": 1.0,
    "default_pose": [0.0, 0.8, -1.5]
  },
  "termination": {
    "minimum_base_height": 0.18,
    "maximum_tilt_degrees": 70
  }
}
```

The real manifest must list every joint explicitly rather than relying on an
abbreviated order. Loading fails before simulation if dimensions, joint names,
control ranges, or declared frequencies do not match the MuJoCo model.

## Runtime components

### Package loader

Responsibilities:

- Mount MJCF and assets into MuJoCo's virtual filesystem
- Load and validate `policy.json`
- Create the MuJoCo model/data objects
- Initialize the ONNX inference session
- Validate input/output tensor names and shapes
- Resolve declared joint and actuator names to MuJoCo indices
- Return actionable errors instead of partially starting the simulation

Start with bundled packages compiled by Vite. User-uploaded packages should
come later, after the manifest contract is stable.

### Inference runtime

Use ONNX Runtime Web. Prefer WebGPU when available and fall back to WASM.

Requirements:

- Cache tensor allocations where possible
- Warm up the model before enabling Play
- Report inference time and selected execution provider
- Keep inference deterministic for a fixed initial state and command sequence
- Reject policies whose tensors do not match the manifest

### Observation adapter

The adapter converts MuJoCo state into the exact ordered float vector expected
by the policy. Supported terms for the first policy should be limited to what
that policy needs:

- Base angular velocity in the robot frame
- Projected gravity in the robot frame
- User velocity command
- Joint position relative to the default pose
- Joint velocity
- Previous action

Each term should be implemented independently and unit-tested. The adapter
should also expose named slices so the UI can inspect observations without
reverse-engineering a flat tensor.

### Action adapter

The first policy should output normalized joint-position offsets:

```text
target = default_pose + action_scale × clipped_action
```

The targets then feed the actuators declared in the manifest. Validate every
target against model control and joint ranges. Record both the raw network
output and applied control so clipping or scaling mistakes are visible.

### Scheduler

Maintain separate counters for:

- Browser render frequency
- MuJoCo physics frequency
- Policy inference frequency
- Diagnostic sampling frequency

Pause stops all simulation advancement. Single-step advances one policy period,
including all contained physics substeps. Slow motion changes wall-clock
playback, not simulation timestep or policy frequency.

### Disturbances

The first disturbance tool applies a short external force to the torso through
MuJoCo's external-force buffer. The UI exposes direction, magnitude, and
duration with conservative bounds. Each disturbance is recorded on the
diagnostic timeline so a failure can be related to the exact perturbation.

### Diagnostics

Keep a bounded ring buffer rather than unbounded frame history. Record at least:

- Simulation time
- Command vector
- Raw action and applied control
- Base height and orientation
- Joint position and velocity
- Contacting body/geom pairs
- Reward terms and total reward
- Policy inference duration
- Disturbance events
- Termination reason

The first UI can display compact sparklines and current values. Full rewind and
policy comparison should be a later milestone because they require state
snapshots and deterministic replay.

## Reward and termination

Inference does not inherently provide reward information. Reward terms must be
implemented in mjplay or declared by a compatible environment definition.

For the bundled Go2 demo, implement a small transparent evaluator:

- Velocity tracking
- Yaw-rate tracking
- Upright orientation
- Action smoothness penalty
- Joint-limit penalty
- Unexpected collision penalty

This evaluator is for debugging and visualization. It must not be presented as
the original training reward unless it exactly reproduces the source
environment.

Termination should pause the run and state the reason, such as low base height,
excessive tilt, non-finite state, or manifest/runtime failure.

## Milestones

### P0 — Policy runtime probe

- Add ONNX Runtime Web.
- Load a tiny test network in the browser.
- Run inference and verify its output against a Node-side reference.
- Measure warm and cold inference time.

**Done:** deterministic output is produced in the browser with no simulation.

### P1 — One complete Go2 policy package

- Bundle the Go2 MJCF, assets, ONNX policy, and manifest.
- Implement only the observation/action terms required by that policy.
- Validate all tensor dimensions and actuator mappings.
- Run inference at the declared control frequency.

**Done:** Go2 stands and walks forward for a repeatable evaluation window.

### P2 — Interactive policy lab

- Add velocity commands.
- Add play, pause, reset, single-step, and slow motion.
- Add directional push disturbances.
- Show current contacts, actions, base state, and inference time.

**Done:** a user can command and perturb the robot without opening developer
tools.

### P3 — Failure diagnostics

- Add bounded timelines for actions, contacts, commands, and reward terms.
- Add explicit termination reasons.
- Highlight clipped actions, joint limits, and unexpected contacts.
- Export a small JSON run report.

**Done:** after a fall, the UI identifies when it happened and presents the
signals most likely to explain it.

### P4 — Policy portability

- Stabilize and version the manifest schema.
- Add a packaged-policy importer.
- Produce a validation report before loading.
- Document how to export a compatible policy.

**Done:** a second policy package can be added without changing core runtime
code.

## Testing strategy

- Unit-test every observation term and action transform against fixed arrays.
- Compare browser ONNX outputs to a Node or Python reference for known inputs.
- Test manifest failures: missing joints, wrong tensor sizes, bad frequencies,
  incompatible actuator types, and non-finite normalization constants.
- Run deterministic rollouts from a fixed keyframe and compare summary metrics.
- Keep the existing MuJoCo model/contact probes.
- Browser-test controls, pause/step semantics, model rendering, and WebGPU→WASM
  fallback.
- Track median and worst-case inference time over a fixed run.

## Performance targets

- Render near 60 fps on a current desktop browser.
- Keep median policy inference below one policy period.
- Avoid allocations inside the physics loop where practical.
- Cap diagnostic memory with fixed-size ring buffers.
- Load only the selected robot and policy assets rather than bundling every
  package eagerly.

The current application eagerly imports all model assets. Policy work should
first introduce lazy per-package loading so adding Go2 and an ONNX model does
not make every initial page load significantly larger.

## Security and failure handling

- Treat uploaded manifests and filenames as untrusted input.
- Do not permit arbitrary JavaScript policy adapters.
- Restrict manifests to known observation and action term types.
- Bound tensor sizes, history lengths, frequencies, and disturbance magnitude.
- Abort on non-finite observations, actions, controls, or MuJoCo state.
- Dispose MuJoCo/Embind and ONNX resources when switching packages.
- Show validation failures in the UI with the exact field and expected value.

## Explicitly deferred

- Arbitrary Python policy execution
- Training or fine-tuning in the browser
- A universal adapter for undocumented ONNX policies
- Remote policy-serving infrastructure
- Multi-robot simulation
- RL environment authoring
- Long-term trajectory storage
- Full rewind and branching simulation
- Side-by-side policy comparison before deterministic replay exists

## Immediate next implementation step

### Current implementation status

P0 and the core P1 policy path are complete. mjplay now includes:

- ONNX Runtime Web loaded only when Policy mode is selected
- A versioned and validated policy manifest
- Exact actuator-name resolution against the MuJoCo model
- A deterministic ONNX fixture with browser and Node reference checks
- Separate 20 Hz policy and MuJoCo physics scheduling
- Play, pause, reset, single-policy-step, amplitude, speed, and push controls
- Live execution provider, cold-start time, inference time, action norm, and
  clipping diagnostics
- A UR5e neural-motion demo that visibly exercises all six actuators
- A pinned community Go2 PPO checkpoint with its split ONNX weights
- The checkpoint's exact 45-value observation and 12-action contract
- Body-frame angular velocity, projected gravity, velocity commands, relative
  joint position/velocity, and previous-action observation adapters
- A 50 Hz policy loop feeding a per-physics-step PD torque controller
- Forward, lateral, and yaw commands plus a torso push disturbance
- MuJoCo Menagerie Go2 assets and explicit BSD-3-Clause attribution

The Go2 checkpoint is a third-party community model by Hugging Face user
`diasAiMaster`, pinned to revision
`9a723b7ab0784cd86abb942836aa1f208ddde891`. It is declared BSD-3-Clause and
was trained with Unitree's `unitree_rl_mjlab`, but it is **not an official
Unitree checkpoint**. Its original model card and deployment YAML are preserved
in the policy package.

### Original P0 checklist

Implement P0 before adding another robot:

1. Add ONNX Runtime Web as a dependency.
2. Create a tiny deterministic ONNX fixture.
3. Load it lazily in the browser.
4. Run a fixed input through it.
5. Display execution provider, output, cold-start time, and warm inference time.
6. Add a headless reference check.

This de-risks browser inference and establishes the resource-loading pattern
needed for the real Go2 policy without prematurely committing to a policy
manifest or observation convention.
