import { duration } from "../common/duration";
import { fork } from "../common/fork";

interface ICacheEntryMetadata<Value> {
  generating: boolean;
  expiration: Date;
  lastValue?: Value;
}

type CacheKey = string;

export class StoreCache<Key, Value> {
  private entries = new Map<CacheKey, Value>();
  private metadata = new Map<CacheKey, ICacheEntryMetadata<Value>>();
  private capacity!: number;
  private timeout!: number;

  constructor({
    capacity,
    timeout,
  }: { capacity?: number; timeout?: number } = {}) {
    this.capacity = capacity || 10000;
    this.timeout = timeout || duration.minute;
  }

  public get(key: Key, generator?: () => Promise<Value>): Value | undefined {
    const cacheKey = this.keyToCacheKey(key);
    const entry = this.entries.get(cacheKey);
    const metadata = this.metadata.get(cacheKey);

    if (
      (entry === undefined ||
        (metadata && metadata?.expiration.getTime() < Date.now())) &&
      !metadata?.generating &&
      generator
    ) {
      this.generate(key, generator());
    }

    if (entry === undefined && metadata && metadata.lastValue !== undefined) {
      return metadata.lastValue;
    }

    return entry;
  }

  public set(key: Key, value: Value) {
    const cacheKey = this.keyToCacheKey(key);
    this.metadata.set(cacheKey, {
      generating: false,
      expiration: new Date(Date.now() + this.timeout),
      lastValue: value,
    });
    this.entries.set(cacheKey, value);

    if (this.metadata.size > this.capacity) {
      this.deleteOldestEntry();
    }
  }

  public clear() {
    this.entries.clear();
    [...this.metadata.values()].forEach((value) => (value.generating = false));
  }

  public clearKey(key: string): void {
    this.metadata.delete(key);
    this.entries.delete(key);
  }

  private keyToCacheKey(key: Key): CacheKey {
    return JSON.stringify(key);
  }

  private deleteOldestEntry() {
    if (this.metadata.size < 1) {
      return;
    }
    const [key] = [...this.metadata.entries()].reduce(
      ([oldestKey, oldestEntry], [thisKey, entry]) =>
        entry.expiration.getTime() < oldestEntry.expiration.getTime()
          ? [thisKey, entry]
          : [oldestKey, oldestEntry]
    );
    this.clearKey(key);
  }

  private generate(key: Key, promise: Promise<Value>) {
    const cacheKey = this.keyToCacheKey(key);
    const existingMetadata = this.metadata.get(cacheKey) || {};
    this.metadata.set(cacheKey, {
      ...existingMetadata,
      generating: true,
      expiration: new Date(Date.now() + this.timeout),
    });
    setTimeout(() => {
      fork(
        promise.then((value) => {
          const metadata = this.metadata.get(cacheKey);
          const canceled = !metadata?.generating;
          if (!canceled) {
            this.set(key, value);
          }
        })
      );
    }, 0);
  }
}
