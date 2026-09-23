import { defineConfig } from 'vite';
import { cp } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';

export default defineConfig({
  base: process.env.GITHUB_ACTIONS ? '/6dof-gym-web/' : '/',
  build: {
    rollupOptions: {
      input: {
        main: fileURLToPath(new URL('./index.html', import.meta.url)),
        cloth: fileURLToPath(new URL('./cloth.html', import.meta.url)),
      },
    },
  },
  plugins: [{
    name: 'copy-public-data',
    async closeBundle() {
      await cp(new URL('./data', import.meta.url), new URL('./dist/data', import.meta.url), { recursive: true });
    },
  }],
});
