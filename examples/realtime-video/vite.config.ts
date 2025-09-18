  import { defineConfig } from 'vite';
  import react from '@vitejs/plugin-react';
  import { nodePolyfills } from 'vite-plugin-node-polyfills';

  export default defineConfig({
    plugins: [
      react(),
        nodePolyfills({
    include: ['process', 'buffer', 'util', 'timers', 'events', 'stream', 'crypto'],
    globals: {
      Buffer: true,
      process: true,
      setImmediate: true,
      clearImmediate: true,
    },
    protocolImports: true,
  }),
    ],
  });
