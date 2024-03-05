import { duration } from "../../../common/duration";

interface ICacheEntryMetadata<Value> {
  generating: Promise<Value> | false;
  staleAt: number;
}

type CacheKey = string;

export class StoreCache<Key, Value> {
  private entries = new Map<CacheKey, Value>();
  private metadata = new Map<CacheKey, ICacheEntryMetadata<Value>>();
  private capacity!: number;
  private staleIntervalMs!: number;

  constructor({
    capacity,
    timeout,
  }: { capacity?: number; timeout?: number } = {}) {
    this.capacity = capacity || 10000;
    this.staleIntervalMs = timeout || duration.minute;
  }

  public get(key: Key, generator?: () => Promise<Value>): Value | undefined {
    const cacheKey = this.keyToCacheKey(key);

    if (this.isStale(cacheKey) && !this.isGenerating(cacheKey) && generator) {
      void this.generate(key, generator);
    }
    return this.entries.get(cacheKey);
  }

  public set(key: Key, value: Value) {
    const cacheKey = this.keyToCacheKey(key);
    this.metadata.set(cacheKey, {
      generating: false,
      staleAt: performance.now() + this.staleIntervalMs,
    });
    const previousValue = this.entries.get(cacheKey);
    const isDuplicate = JSON.stringify(previousValue) === JSON.stringify(value);
    if (!isDuplicate) {
      this.entries.set(cacheKey, value);
      this.enforceMaxSize();
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

  private enforceMaxSize(): void {
    while (this.metadata.size > this.capacity && this.metadata.size > 0) {
      const [key] = [...this.metadata.entries()].reduce(
        ([oldestKey, oldestEntry], [thisKey, entry]) =>
          entry.staleAt < oldestEntry.staleAt
            ? [thisKey, entry]
            : [oldestKey, oldestEntry]
      );
      this.clearKey(key);
    }
  }

  private isStale(cacheKey: CacheKey): boolean {
    const metadata = this.metadata.get(cacheKey);
    return metadata ? metadata?.staleAt < performance.now() : true;
  }

  private isGenerating(cacheKey: CacheKey): Promise<Value> | false {
    const metadata = this.metadata.get(cacheKey);
    return metadata ? metadata.generating : false;
  }

  private generate(key: Key, generator: () => Promise<Value>) {
    const cacheKey = this.keyToCacheKey(key);
    const existingMetadata = this.metadata.get(cacheKey) || {};

    const generating = generator()
      .then((value) => {
        const metadata = this.metadata.get(cacheKey);
        const canceled = metadata?.generating !== generating;
        if (!canceled) {
          this.set(key, value);
        }
        return value;
      })
      .catch((err) => {
        this.metadata.delete(cacheKey);
        throw err;
      });

    this.metadata.set(cacheKey, {
      ...existingMetadata,
      generating,
      staleAt: performance.now() + this.staleIntervalMs,
    });
  }
}
