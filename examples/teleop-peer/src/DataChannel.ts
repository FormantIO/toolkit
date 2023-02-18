export type DataChannelStringListener = (message: string) => void;
export type DataChannelBinaryListener = (message: Uint8Array) => void;
export type DataChannelListener = () => void;
export type DataChannelErrorListener = (ev: Event) => void;
export class DataChannel {
  ready = false;
  listeners: DataChannelStringListener[] = [];
  openListeners: DataChannelListener[] = [];
  closeListeners: DataChannelListener[] = [];
  errorListeners: DataChannelErrorListener[] = [];
  binaryListeners: DataChannelBinaryListener[] = [];
  error: string | undefined;
  decoder = new TextDecoder();

  constructor(private dataChannel: RTCDataChannel) {
    this.dataChannel.binaryType = "arraybuffer";
    this.dataChannel.onopen = () => {
      this.ready = true;
      this.openListeners.forEach((listener) => listener());
    };
    this.dataChannel.onclose = () => {
      this.ready = false;
      this.closeListeners.forEach((listener) => listener());
    };
    this.dataChannel.onerror = (e) => {
      console.error(e);
      this.error = "An error occurred in DataChannel";
      this.errorListeners.forEach((listener) => listener(e));
    };
    this.dataChannel.onmessage = (m: MessageEvent) => {
      this.listeners.forEach((_) => {
        const d = new Uint8Array(m.data);
        const s = this.decoder.decode(d);
        _(s);
      });
      this.binaryListeners.forEach((_) => {
        _(new Uint8Array(m.data));
      });
    };
  }

  addOpenListener(listener: DataChannelListener) {
    this.openListeners.push(listener);
  }

  removeOpenListener(listener: DataChannelListener) {
    this.openListeners = this.openListeners.filter((_) => _ !== listener);
  }

  addCloseListener(listener: DataChannelListener) {
    this.closeListeners.push(listener);
  }

  removeCloseListener(listener: DataChannelListener) {
    this.closeListeners = this.closeListeners.filter((l) => l !== listener);
  }

  addErrorListener(listener: DataChannelErrorListener) {
    this.errorListeners.push(listener);
  }

  removeErrorListener(listener: DataChannelErrorListener) {
    this.errorListeners = this.errorListeners.filter((l) => l !== listener);
  }

  async waitTilReady(): Promise<boolean> {
    if (this.ready) {
      return true;
    }
    const p = new Promise<boolean>((resolve, reject) => {
      let a = window.setInterval(() => {
        if (this.ready) {
          window.clearInterval(a);
          resolve(true);
        }
        if (this.error) {
          reject(this.error);
        }
      }, 10);
    });
    return p;
  }

  send(data: string) {
    if (!this.ready) {
      throw new Error("Connection has been closed");
    }
    this.dataChannel.send(data);
  }

  sendBinary(data: Uint8Array) {
    if (!this.ready) {
      throw new Error("Connection has been closed");
    }
    this.dataChannel.send(data);
  }

  addListener(listener: DataChannelStringListener) {
    this.listeners.push(listener);
  }

  removeListener(listener: DataChannelStringListener) {
    const i = this.listeners.indexOf(listener);
    if (i === -1) {
      throw new Error("Could not find data channel listener to remove");
    }
    if (this.error) {
      throw new Error(this.error);
    }
    this.listeners.splice(i, 1);
  }

  addBinaryListener(listener: DataChannelBinaryListener) {
    this.binaryListeners.push(listener);
  }

  removeBinaryListener(listener: DataChannelBinaryListener) {
    const i = this.binaryListeners.indexOf(listener);
    if (i === -1) {
      throw new Error("Could not find data channel listener to remove");
    }
    if (this.error) {
      throw new Error(this.error);
    }
    this.binaryListeners.splice(i, 1);
  }
}
