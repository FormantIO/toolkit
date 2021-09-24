export type DataChannelListener = (message: any) => void;

export class DataChannel {
  ready = false;
  listeners: DataChannelListener[] = [];
  error: string | undefined;
  decoder = new TextDecoder();
  constructor(private dataChannel: RTCDataChannel) {
    this.dataChannel.onopen = () => {
      this.ready = true;
    };
    this.dataChannel.onclose = () => {
      this.ready = false;
    };
    this.dataChannel.onerror = (e) => {
      console.error(e);
      this.error = "An error occurred in DataChannel";
    };
    this.dataChannel.onmessage = (m: MessageEvent) => {
      const d = new Uint8Array(m.data);
      var s = this.decoder.decode(d);
      this.listeners.forEach((_) => _(s));
    };
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

  addListener(listener: DataChannelListener) {
    this.listeners.push(listener);
  }

  removeListener(listener: DataChannelListener) {
    const i = this.listeners.indexOf(listener);
    if (i === -1) {
      throw new Error("Could not find data channel listener to remove");
    }
    if (this.error) {
      throw new Error(this.error);
    }
    this.listeners.splice(i, 1);
  }
}
