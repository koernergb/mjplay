# mjplay — Task Cards

**Project:** Browser MJCF/URDF viewer + contact playground.
**Stack:** `@mujoco/mujoco@3.10.0` (official DeepMind WASM, verified) · Three.js · Vite · MuJoCo Menagerie models.

**Completion gate (the whole point):**
> Load a Menagerie robot, drag a joint slider, and when two bodies collide the contacting geometry turns red — 60fps in-browser, with a reset button.

**API verified against 3.10.0 type defs** (not assumed): `data.ncon`, `data.contact.get(i)` → `MjContact{geom1, geom2}`, `model.geom_bodyid`, `model.njnt`, `model.jnt_range`, `model.jnt_type`, `data.qpos`, `data.ctrl`, `model.nu`, `model.nq`.

Every card has a binary DONE check. If you can't tick it, the card isn't finished. Do them in order. **Stop at any milestone boundary and you still have a shippable thing.**

---

## M0 — Contact probe + viewer boots  ·  GO/NO-GO

The point of M0 is to prove the risky assumption (contacts readable from JS) *before* building any UI. Do NOT start M1 until M0.4 prints numbers.

### M0.1 — Project skeleton
- Vite vanilla-TS project. `npm i @mujoco/mujoco three`.
- Dev server serves over HTTP (WASM won't load from `file://`). Confirm `vite dev` opens a blank page with no console errors.
- **DONE:** blank page loads, zero console errors.

### M0.2 — Load MuJoCo + a model
- `await load_mujoco()`, mount MEMFS, write a Menagerie MJCF (start with `franka_emika_panda/scene.xml` — it has a floor, so contacts are easy to trigger).
- `new mujoco.Model(...)`, `new mujoco.State(model)`, `new mujoco.Simulation(model, state)`.
- Log `model.nbody`, `model.njnt`, `model.nq`, `model.nu`.
- **DONE:** those four counts print non-zero for the Panda.

### M0.3 — Render one frame in Three.js
- Walk geoms, build Three meshes from geom type/size (box/sphere/capsule/cylinder/mesh). Position from `data.xpos`/`data.xmat` after one `mj_forward`.
- OrbitControls, a light, a ground plane.
- **DONE:** the Panda is visibly recognizable on screen, orbitable.

### M0.4 — CONTACT PROBE (the go/no-go)
- Step the sim in a loop (`mj_step`). Let the robot settle/drop onto the floor.
- Each step: `console.log(data.ncon)`; for `i` in `0..ncon`, `const c = data.contact.get(i); console.log(c.geom1, c.geom2, c.dist)`.
- **DONE:** when the robot touches the floor, `ncon > 0` and geom IDs print. ← **IF THIS FAILS, STOP AND REPLAN.** Everything downstream depends on it. (It shouldn't fail — API is verified — but verify the *values* are sane, not just present.)

---

## M1 — Joint dragging  ·  COMPLETE

After M1 you have a working URDF/MJCF inspector. That alone is portfolio-worthy. Ship here if you must.

### M1.1 — Sim loop with fixed timestep ✅
- rAF loop, accumulator stepping `mj_step` at `model.opt.timestep`, render interpolated. Reset button → `mj_resetData`.
- **DONE:** robot falls under gravity smoothly at 60fps; reset returns it to start.

### M1.2 — Auto-generate joint sliders ✅
- For each joint `j` in `0..model.njnt`: read `jnt_type[j]` (skip free/ball for v1 — handle hinge & slide only), `jnt_range[2*j]`/`[2*j+1]` for min/max. Build one HTML slider each, labeled by joint index.
- **DONE:** N sliders appear for an N-actuated-joint robot, each with correct min/max.

### M1.3 — Slider → pose ✅
- Slider writes to the joint's `qpos` address, then `mj_forward` (kinematics only, no dynamics) so dragging poses the robot without it falling. Toggle: "pose mode" (forward only) vs "sim mode" (full step).
- **DONE:** drag a slider in pose mode → that joint rotates, rest of robot stays put.

---

## M2 — Red on contact  ·  CORE COMPLETE

### M2.1 — geom → mesh map ✅
- Build a lookup: geom index → Three mesh (you already create meshes per geom in M0.3; store the mapping there). Also cache each mesh's original material.
- **DONE:** `geomToMesh[k]` returns the right mesh for any geom `k`.

### M2.2 — Per-frame contact highlight ✅
- Each frame after stepping: collect the set of geoms in contact (`c.geom1`, `c.geom2` for `i in 0..ncon`). Set those meshes' material to red; revert everything else to cached original.
- Do it as a diff (only touch meshes whose state changed) to hold 60fps.
- **DONE:** self-collide the robot (or land it on the floor) → exactly the touching geoms turn red, revert when clear. **← COMPLETION GATE MET.**

### M2.3 — Polish to demo-ready (optional, timeboxed to 2h)
- Model picker dropdown (2–3 Menagerie robots). "Reset" + "pose/sim" toggle in a clean corner panel.
- README with the one-line thesis hook + a GIF.
- **DONE:** a stranger can load the page, pick a robot, drag it into a self-collision, and see red — without instructions.

---

## OUT OF SCOPE — do not build (these are the escape hatches)
- ❌ Perfect URDF import fidelity — Menagerie MJCF only; URDF best-effort
- ❌ Contact *forces*/friction viz — boolean touch only (you have `dist`/`mu` if tempted; resist)
- ❌ Free/ball joint sliders in v1 (hinge + slide only)
- ❌ In-browser model editing/authoring
- ❌ Multiple robots in one scene, scene building
- ❌ Save/load/share state
- ❌ Your own emscripten compile — the npm build is verified working
- ❌ Trajectory playback, control policies, "and then RL"
- ❌ Multithreaded (`mt/`) build — single-threaded is enough for one robot

## Thesis hook (README, one line — don't build on it)
> Static URDF linters tell you a file *parses*. mjplay tells you what the robot *does* — where it self-collides and where joint limits bite. Interactive front-end to the robotics-tooling-gap thesis.
