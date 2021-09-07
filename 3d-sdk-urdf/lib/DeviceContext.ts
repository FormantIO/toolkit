type ListenerCallback = () => void;

export class DeviceContext {
  listeners: { [key in string]: ListenerCallback[] } = {};
  addRealtimeListener(streamName: string, callback: ListenerCallback) {
    if (!this.listeners[streamName]) {
      this.listeners[streamName] = [];
    }
    this.listeners[streamName].push(callback);
  }
  removeRealtimeListener(streamName: string, callback: ListenerCallback) {
    const listeners = this.listeners[streamName];
    if (!this.listeners[streamName]) {
      throw new Error("no listeners exist");
    }
    const idx = this.listeners[streamName].indexOf(callback);
    if (idx === -1) {
      throw new Error("listener does not exist exist");
    }
    listeners.splice(idx, 1);
  }
}

let defaultContext: DeviceContext | undefined;

export function getDefaultContext(): DeviceContext {
  if (!defaultContext) {
    defaultContext = new DeviceContext();
  }
  return defaultContext;
}
