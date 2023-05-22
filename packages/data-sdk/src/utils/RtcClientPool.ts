import { RtcClient, IRtcClientConfiguration } from "@formant/realtime-sdk";

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
  private readonly proxyHandler: ProxyHandler<RtcClient>;
  private proxyReceivers: Map<RtcClient, ReceiveFn | null> = new Map();
  private teardownTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(options: IRtcClientPoolOptions) {
    const { createClient, ttl = 0 } = options;
    this.createClient = createClient;
    this.ttl = Math.max(ttl, 0);
    this.proxyHandler = {
      get: (target, prop, receiver) => {
        switch (prop) {
          case "shutdown":
            return () => this.releaseInstance(receiver);
          default:
            return Reflect.get(target, prop, receiver);
        }
      },
    };
  }

  get isActive(): boolean {
    return this[singleton] !== null;
  }

  get size(): number {
    return this.proxyReceivers.size;
  }

  get(onReceive?: ReceiveFn): RtcClient {
    const proxy = new Proxy(this.allocate(), this.proxyHandler);
    this.proxyReceivers.set(proxy, onReceive ?? null);
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
    this.proxyReceivers.forEach((it) => it?.(peerId, message));
  };

  private async releaseInstance(proxy: RtcClient): Promise<boolean> {
    if (!this.proxyReceivers.delete(proxy)) {
      console.warn("this instance has already been released!");
      return false;
    }

    if (this.proxyReceivers.size !== 0) {
      return false;
    }

    if (!this.teardownTimeout && Number.isFinite(this.ttl)) {
      if (this.ttl === 0) {
        await this.teardown();
      } else {
        this.teardownTimeout = setTimeout(() => {
          this.teardown()
            .catch((err) => console.error("teardown failed", { err }))
            .finally(() => (this.teardownTimeout = null));
        }, this.ttl);
      }
    }
    return true;
  }
}
