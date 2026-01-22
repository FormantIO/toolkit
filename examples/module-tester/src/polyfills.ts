// Polyfills for Node.js globals required by Formant SDK
// This must be imported first before any SDK imports

// Polyfill for process.nextTick
if (typeof process === 'undefined' || typeof process.nextTick === 'undefined') {
  (globalThis as any).process = {
    ...(globalThis as any).process,
    nextTick: (callback: Function, ...args: any[]) => {
      Promise.resolve().then(() => callback(...args));
    },
    env: {},
    platform: 'browser',
    version: '',
  };
}
