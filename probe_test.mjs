// Headless verification of the M0 load-bearing logic against the real WASM,
// mirroring exactly what main.js does: load XML, step, read contacts.
import loadMujoco from '@mujoco/mujoco';
import { SCENE_XML } from './src/scene.js';

const mujoco = await loadMujoco();
console.log('module loaded, mujoco keys sample:',
  Object.keys(mujoco).filter(k => /^Mj|^mj_/.test(k)).slice(0, 8).join(', '), '...');

const model = mujoco.MjModel.from_xml_string(SCENE_XML);
const data  = new mujoco.MjData(model);
console.log(`[M0.2] nbody=${model.nbody} njnt=${model.njnt} ngeom=${model.ngeom} nq=${model.nq} nu=${model.nu}`);

// sanity: geom typed-array views readable?
console.log('[M0.3] geom_type[0..ngeom]=',
  Array.from({length: model.ngeom}, (_, g) => model.geom_type[g]).join(','));

// step until first contact
const dt = model.opt.timestep;
mujoco.mj_forward(model, data);
let firstContactStep = -1, maxNcon = 0;
for (let step = 0; step < 2000; step++) {
  mujoco.mj_step(model, data);
  const ncon = data.ncon;
  maxNcon = Math.max(maxNcon, ncon);
  if (ncon > 0) {
    if (firstContactStep < 0) {
      firstContactStep = step;
      console.log(`[M0.4] FIRST CONTACT at step ${step} (t=${data.time.toFixed(3)}s), ncon=${ncon}`);
      const contacts = data.contact;
      for (let i = 0; i < ncon; i++) {
        const c = contacts.get(i);
        console.log(`  contact ${i}: geom1=${c.geom1} geom2=${c.geom2} dist=${c.dist.toFixed(5)}`);
        c.delete();
      }
      contacts.delete();
    }
  }
}
console.log(`[M0.4] max ncon over run = ${maxNcon}, final t=${data.time.toFixed(2)}s`);
console.log(firstContactStep >= 0
  ? '[M0.4] ✓ GO — contacts readable, values sane.'
  : '[M0.4] ✗ NO-GO — no contacts detected, replan.');
data.delete(); model.delete();
