// Headless verification for the bundled Menagerie UR5e model.
import fs from 'node:fs/promises';
import path from 'node:path';
import loadMujoco from '@mujoco/mujoco';

const mujoco = await loadMujoco();
const sourceRoot = path.resolve('src/models/ur5e');
const virtualRoot = '/models/ur5e';
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

if (model.njnt !== 6 || model.nkey < 1 || model.ngeom < 30) {
  throw new Error(`Unexpected UR5e structure: ${model.nbody} bodies, ${model.njnt} joints, ${model.ngeom} geoms`);
}

mujoco.mj_resetDataKeyframe(model, data, 0);
const elbowAddress = model.jnt_qposadr[2];
data.qpos[elbowAddress] = 2.1;
mujoco.mj_forward(model, data);
if (Math.abs(data.qpos[elbowAddress] - 2.1) > 1e-6) throw new Error('UR5e elbow pose edit did not stick');

console.log(`[UR5e] ${model.nbody} bodies, ${model.njnt} joints, ${model.ngeom} geoms`);
console.log(`[UR5e] elbow posed at ${data.qpos[elbowAddress].toFixed(2)} rad; ncon=${data.ncon}`);
console.log('[UR5e] ✓ Menagerie assets, joint metadata, pose editing, and contacts are readable.');

data.delete();
model.delete();
