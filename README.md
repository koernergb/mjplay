# mjplay

Browser MJCF/URDF viewer + contact playground. Load a robot, drag its joints,
watch contacting geometry light up red in real time. `urdfcheck` with a face.

> Static URDF linters tell you a file *parses*. mjplay tells you what the robot
> *does* — where it self-collides and where joint limits bite.

## Status: M0 complete (verified)

The contact-readability go/no-go is **green**, verified headless against the
real `@mujoco/mujoco` 3.10.0 WASM build (see `probe_test.mjs`):
first contact fires at t≈0.35s and per-contact geom IDs are readable from JS.

## Run

```sh
npm install
npm run dev
```

Open the page: three primitive bodies fall onto a floor + obstacle. The HUD
shows live `ncon`; it turns red when contacts exist. Contacting geoms turn red
in the 3D view. Open the console for the one-time first-contact geom-ID dump.

## Headless probe (no browser)

```sh
node probe_test.mjs
```

Runs the exact load → step → read-contacts path main.js uses. Use this to
re-verify the API after any dependency bump.

## Stack

- `@mujoco/mujoco` 3.10.0 — official DeepMind WASM build, single-threaded
  (no cross-origin-isolation headers needed). macOS-supported.
- Three.js — rendering. Meshes built per-geom from primitive types.
- Vite — dev server; copies the `.wasm` asset to the served root.

## Gotchas already handled

- `data.contact` returns a **copy** each step — re-fetched and `.delete()`d
  every frame (Embind handles are not GC'd).
- MuJoCo capsule/cylinder long-axis is local **Z**; Three's is local **Y** — rotated.
- MuJoCo `geom_xmat` is **row-major** 3×3 — transposed into Three's Matrix4.

## Next (see TASKS.md)

- **M1** — swap in a Menagerie MJCF; auto-generate joint sliders from
  `jnt_type`/`jnt_range` (hinge + slide only); pose vs sim toggle.
- **M2** — the red-on-contact highlight is already wired; polish + model picker.

## Out of scope

Contact forces, friction viz, free/ball-joint sliders, in-browser model
editing, multi-robot scenes, your own emscripten compile. See TASKS.md.
