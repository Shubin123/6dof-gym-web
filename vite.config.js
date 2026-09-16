import { defineConfig } from 'vite';
import { cp } from 'node:fs/promises';

export default defineConfig({
  base: process.env.GITHUB_ACTIONS ? '/6dof-gym-web/' : '/',
  plugins: [{
    name: 'copy-public-data',
    async closeBundle() {
      await cp(new URL('./data', import.meta.url), new URL('./dist/data', import.meta.url), { recursive: true });
    },
  }],
});
