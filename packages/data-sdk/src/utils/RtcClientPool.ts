import {
  RtcClient,
  RtcClientV1,
  IRtcClientConfiguration,
} from "@formant/realtime-sdk";

type ReceiveFn = IRtcClientConfiguration["receive"];
type CreateClientFn<T extends RtcClient | RtcClientV1> = (
  receive: ReceiveFn
) => T;

export interface IRtcClientPoolOptions<T extends RtcClient | RtcClientV1> {
  createClient: CreateClientFn<T>;
  ttlMs?: number;
}

const singleton = Symbol("RtcClientPool.instance");

export class RtcClientPool<T extends RtcClient | RtcClientV1> {
  [singleton]: T | null = null;

  private readonly createClient: CreateClientFn<T>;
  private readonly ttlMs: number;
  private readonly proxyHandler: ProxyHandler<T>;
  private proxyReceivers: Map<T, ReceiveFn | null> = new Map();
  private teardownTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(options: IRtcClientPoolOptions<T>) {
    const { createClient, ttlMs = 0 } = options;
    this.createClient = createClient;
    this.ttlMs = Math.max(ttlMs, 0);
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

  get(onReceive?: ReceiveFn): T {
    const proxy = new Proxy(this.allocate(), this.proxyHandler);
    this.proxyReceivers.set(proxy, onReceive ?? null);
    return proxy;
  }

  private allocate(): T {
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

    try {
      await instance.shutdown();
    } finally {
      this[singleton] = null;
    }
  }

  private dispatch: ReceiveFn = (peerId, message) => {
    this.proxyReceivers.forEach((it) => it?.(peerId, message));
  };

  private async releaseInstance(proxy: T): Promise<boolean> {
    if (!this.proxyReceivers.delete(proxy)) {
      console.warn("this instance has already been released!");
      return false;
    }

    if (this.proxyReceivers.size !== 0) {
      return false;
    }

    if (!this.teardownTimeout && Number.isFinite(this.ttlMs)) {
      if (this.ttlMs === 0) {
        await this.teardown();
      } else {
        this.teardownTimeout = setTimeout(() => {
          this.teardown()
            .catch((err) => console.error("teardown failed", { err }))
            .finally(() => (this.teardownTimeout = null));
        }, this.ttlMs);
      }
    }
    return true;
  }
}
