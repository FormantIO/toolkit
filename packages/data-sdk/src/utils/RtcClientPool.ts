import { RtcClient, IRtcClientConfiguration } from "@formant/realtime-sdk";

export interface PooledRtcClient extends RtcClient {
  release(): boolean;
}

type ReceiveFn = IRtcClientConfiguration["receive"];
type CreateClientFn = (receive: ReceiveFn) => RtcClient;

export interface IRtcClientPoolOptions {
  createClient: CreateClientFn;
  ttl?: number;
}

const singleton = Symbol("RtcClientPool.instance");

export class RtcClientPool {
  [singleton]: RtcClient | null = null;

  private readonly createClient: CreateClientFn;
  private readonly ttl: number;
  private proxyReceivers: Map<PooledRtcClient, ReceiveFn> = new Map();
  private teardownTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(options: IRtcClientPoolOptions) {
    const { createClient, ttl = 0 } = options;
    this.createClient = createClient;
    this.ttl = Math.max(ttl, 0);
  }

  get size(): number {
    return this.proxyReceivers.size;
  }

  get(onReceive?: ReceiveFn): PooledRtcClient {
    const proxy = new Proxy(this.allocate(), {
      get: (target, prop, receiver) => {
        switch (prop) {
          case "release":
            return () => this.releaseInstance(receiver);
          case "shutdown":
            return () => {
              throw new Error("shutdown not allowed for pooled client");
            };
          default:
            return Reflect.get(target, prop, receiver);
        }
      },
    }) as PooledRtcClient;

    this.proxyReceivers.set(proxy, onReceive ?? (() => {}));
    return proxy;
  }

  private allocate(): RtcClient {
    if (this[singleton]) {
      // cancel any outstanding teardown request/keep this singleton alive
      if (this.teardownTimeout) {
        clearTimeout(this.teardownTimeout);
        this.teardownTimeout = null;
      }

      return this[singleton];
    }

    const client = this.createClient(this.dispatch);
    this[singleton] = client;
    return client;
  }

  private async teardown() {
    const instance = this[singleton];
    if (!instance) {
      console.warn("singleton has already been shutdown!");
      return;
    }

    await instance.shutdown();
    this[singleton] = null;
  }

  private dispatch: ReceiveFn = (peerId, message) => {
    this.proxyReceivers.forEach((it) => it(peerId, message));
  };

  private releaseInstance(proxy: PooledRtcClient) {
    if (!this.proxyReceivers.delete(proxy)) {
      console.warn("this instance has already been released!");
      return false;
    }

    if (this.proxyReceivers.size !== 0) {
      return false;
    }

    if (!this.teardownTimeout && Number.isFinite(this.ttl)) {
      this.teardownTimeout = setTimeout(() => {
        this.teardown()
          .catch((err) => console.error("teardown failed", { err }))
          .finally(() => (this.teardownTimeout = null));
      }, this.ttl);
    }

    return true;
  }
}
