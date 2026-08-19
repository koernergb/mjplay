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

The bundled model files retain their upstream licenses inside each model
directory under `src/models/`.

## Run

```sh
npm install
npm run dev
```

Pick a robot, then use **Pose** to manipulate its supported joints. Switch to
**Simulate** to run dynamics, or **Reset** to restore its home pose.

## Verify

```sh
npm run probe
npm run build
```

The probes validate both the original collision test scene and the bundled
Menagerie models' load → pose → contact paths without a browser.

## Stack

- `@mujoco/mujoco` — official DeepMind WASM bindings
- Three.js — generated primitive and MuJoCo mesh rendering
- Vite — development server and production build

## Current scope

The current release intentionally ships one known-good Menagerie MJCF model.
User-supplied MJCF/URDF loading, perfect URDF fidelity, model authoring, contact
forces, and multi-robot scenes remain out of scope.
