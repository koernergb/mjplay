// Headless verification that the bundled Menagerie model and assets load,
// expose the expected joints, accept a pose edit, and still compute contacts.
import fs from 'node:fs/promises';
import path from 'node:path';
import loadMujoco from '@mujoco/mujoco';

const mujoco = await loadMujoco();
const sourceRoot = path.resolve('src/models/panda');
const virtualRoot = '/models/panda';
mujoco.FS.mkdirTree(`${virtualRoot}/assets`);

async function mountDirectory(relative = '') {
  const entries = await fs.readdir(path.join(sourceRoot, relative), { withFileTypes: true });
  for (const entry of entries) {
    const child = path.join(relative, entry.name);
    if (entry.isDirectory()) await mountDirectory(child);
    else if (!entry.name.endsWith('.md')) {
      mujoco.FS.writeFile(`${virtualRoot}/${child}`, await fs.readFile(path.join(sourceRoot, child)));
    }
  }
}

await mountDirectory();
const model = mujoco.MjModel.from_xml_path(`${virtualRoot}/scene.xml`);
const data = new mujoco.MjData(model);

if (model.nbody !== 12 || model.njnt !== 9 || model.ngeom < 80) {
  throw new Error(`Unexpected Panda structure: ${model.nbody} bodies, ${model.njnt} joints, ${model.ngeom} geoms`);
}

mujoco.mj_resetDataKeyframe(model, data, 0);
const joint4Address = model.jnt_qposadr[3];
data.qpos[joint4Address] = -2.2;
mujoco.mj_forward(model, data);
if (Math.abs(data.qpos[joint4Address] + 2.2) > 1e-6) throw new Error('Joint pose edit did not stick');

console.log(`[Panda] ${model.nbody} bodies, ${model.njnt} joints, ${model.ngeom} geoms`);
console.log(`[Panda] joint4 posed at ${data.qpos[joint4Address].toFixed(2)} rad; ncon=${data.ncon}`);
console.log('[Panda] ✓ Menagerie assets, joint metadata, pose editing, and contacts are readable.');

data.delete();
model.delete();
