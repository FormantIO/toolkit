// Polyfill for setImmediate in browser environments
declare global {
  interface Window {
    setImmediate?: (callback: (...args: any[]) => void, ...args: any[]) => number;
    clearImmediate?: (id: number) => void;
  }
  var setImmediate: (callback: (...args: any[]) => void, ...args: any[]) => number;
  var clearImmediate: (id: number) => void;
}

if (typeof window !== "undefined" && !window.setImmediate) {
  window.setImmediate = function (callback: (...args: any[]) => void, ...args: any[]) {
    return setTimeout(callback, 0, ...args);
  };
  window.clearImmediate = function (id: number) {
    clearTimeout(id);
  };
}

// Make them available globally
if (typeof globalThis !== "undefined") {
  globalThis.setImmediate = globalThis.setImmediate || window?.setImmediate || ((callback: (...args: any[]) => void, ...args: any[]) => setTimeout(callback, 0, ...args));
  globalThis.clearImmediate = globalThis.clearImmediate || window?.clearImmediate || ((id: number) => clearTimeout(id));
}

export {};