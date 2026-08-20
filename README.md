# mjplay

Interactive browser-based MuJoCo robot inspector. Pose a real robot with
model-generated joint controls, run its physics simulation, and see contacting
parts light up red.

> Static URDF linters tell you a file *parses*. mjplay tells you what the robot
> *does* — how its joints move and where it collides.

## Implemented

- Official `@mujoco/mujoco` 3.10.0 WebAssembly runtime
- Franka Emika Panda and Universal Robots UR5e from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
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
roadmap to a real Go2 locomotion policy.

## Verify

```sh
npm run probe
npm run build
```

The probes validate the original collision test scene, both Menagerie models,
manifest validation, deterministic ONNX output, and action adaptation without
a browser.

## Stack

- `@mujoco/mujoco` — official DeepMind WASM bindings
- Three.js — generated primitive and MuJoCo mesh rendering
- Vite — development server and production build

## Current scope

The current release intentionally ships two known-good Menagerie MJCF models.
User-supplied MJCF/URDF loading, perfect URDF fidelity, model authoring, contact
forces, and multi-robot scenes remain out of scope.
