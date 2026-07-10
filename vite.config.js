import { defineConfig } from 'vite';
import { viteStaticCopy } from 'vite-plugin-static-copy';

// The @mujoco/mujoco package ships a .wasm binary that must be served at runtime.
// Vite won't automatically copy it, so we copy it into the served root.
export default defineConfig({
  plugins: [
    viteStaticCopy({
      targets: [
        { src: 'node_modules/@mujoco/mujoco/mujoco.wasm', dest: '.' }
      ]
    })
  ],
  server: { port: 5173, open: true },
  // Keep the wasm out of dependency pre-bundling.
  optimizeDeps: { exclude: ['@mujoco/mujoco'] }
});
