import { defineConfig } from 'vite';
import { nodePolyfills } from 'vite-plugin-node-polyfills';

export default defineConfig({
  plugins: [
    nodePolyfills({
      include: ['process', 'buffer', 'util', 'timers', 'events', 'stream', 'crypto', 'path'],
      globals: {
        Buffer: true,
        process: true,
        setImmediate: true,
        clearImmediate: true,
      },
      protocolImports: true,
    }),
  ],
  define: {
    'process.env.NODE_ENV': JSON.stringify(process.env.NODE_ENV || 'development'),
  },
  optimizeDeps: {
    include: ['@formant/data-sdk'],
  },
});
