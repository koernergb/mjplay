# mjplay

Interactive browser-based MuJoCo robot inspector. Pose a real robot with
model-generated joint controls, run its physics simulation, and see contacting
parts light up red.

> Static URDF linters tell you a file *parses*. mjplay tells you what the robot
> *does* — how its joints move and where it collides.

## Implemented

- Official `@mujoco/mujoco` 3.10.0 WebAssembly runtime
- Franka Emika Panda, Universal Robots UR5e, and Unitree Go2 from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- Detailed mesh rendering in Three.js, including MJCF material colors
- Joint controls generated from MuJoCo joint types, names, ranges, and qpos addresses
- Pose mode for direct hinge/slide joint manipulation
- Simulation mode with fixed-timestep physics
- Reset to the model's Menagerie `home` keyframe
- Live contact count, contact names, and whole-link red highlighting
- Orbit camera and responsive desktop/mobile control panel
- Lazy ONNX Runtime Web policy execution for the UR5e demo
- Policy play, pause, single-step, amplitude/speed commands, push disturbance,
  action norm, clipping, and inference timing
- Community-trained Go2 velocity policy with forward, lateral, and yaw commands
- Exact 45-value Go2 observation adapter and a 50 Hz PD torque-control loop

The bundled model files retain their upstream licenses inside each model
directory under `src/models/`.

## Run

```sh
npm install
npm run dev
```

Pick a robot, then use **Pose** to manipulate its supported joints. Switch to
**Simulate** to run dynamics, or **Reset** to restore its home pose.

On the UR5e, select **Policy** to run the bundled ONNX neural-motion fixture.
The fixture maps phase observations to six actuator targets and exists to prove
the complete browser inference → `data.ctrl` → MuJoCo loop. It is deliberately
not described as a trained robotics policy. See
[`docs/POLICY_EXECUTION.md`](docs/POLICY_EXECUTION.md) for the contract and the
implementation details for browser policy execution.

On the Go2, select **Policy** to run the bundled velocity-flat PPO checkpoint,
then command forward/lateral velocity or yaw and apply a torso push. The policy
is the BSD-3-Clause community model by Hugging Face user `diasAiMaster`, pinned
to revision `9a723b7ab0784cd86abb942836aa1f208ddde891`. It was trained using
Unitree's open-source `unitree_rl_mjlab`, but it is **not an official Unitree
checkpoint**. Its model card, deployment YAML, license, and provenance notice
are retained in `src/policies/go2-velocity-flat/`.

## Verify

```sh
npm run probe
npm run build
```

The probes validate the original collision test scene, all three Menagerie
models, both ONNX policies, the Go2 observation/action adapters, PD control, and
a one-second commanded Go2 rollout without a browser.

## Stack

- `@mujoco/mujoco` — official DeepMind WASM bindings
- Three.js — generated primitive and MuJoCo mesh rendering
- Vite — development server and production build

## Current scope

The current release intentionally ships three known-good Menagerie MJCF models.
User-supplied MJCF/URDF loading, perfect URDF fidelity, model authoring, contact
forces, and multi-robot scenes remain out of scope.
