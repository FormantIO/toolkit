import stringify from "fast-json-stable-stringify";
import BaseLruCache from "lru-cache";

export type CacheOptions<V> = BaseLruCache.Options<string, V> & {
  name: string;
  fastStringify?: boolean;
};

export class LruCache<K, V> {
  private cache!: BaseLruCache<string, V>;
  private stringify: (_: K) => string;

  constructor(private options: CacheOptions<V>) {
    this.cache = new BaseLruCache<string, V>({
      // if using the dispose callback,
      // by default automatically prune expired entries so
      // they are delivered consistently and quickly
      ...(options.dispose || options.disposeAfter
        ? { ttlAutopurge: true }
        : {}),
      ...options,
      dispose: (...args) => {
        options.dispose?.(...args);
      },
      disposeAfter: (...args) => {
        options.disposeAfter?.(...args);
      },
    });
    this.stringify = options.fastStringify ? JSON.stringify : stringify;
  }

  public set(key: K, value: V, ttl?: number): void {
    const keyString = this.stringify(key);
    if (!this.cache.set(keyString, value, { ttl })) {
      const size = this.cache.sizeCalculation
        ? this.cache.sizeCalculation(value, keyString)
        : "unknown";
      throw Error(`Value too large (${size} > ${this.cache.max})`);
    }
  }

  public get(key: K): V | undefined {
    const keyString = this.stringify(key);
    return this.cache.get(keyString);
  }

  public delete(key: K) {
    this.cache.delete(this.stringify(key));
  }

  public peek(key: K): V | undefined {
    return this.cache.peek(this.stringify(key));
  }

  public size() {
    return this.cache.size;
  }

  public clear() {
    this.cache.clear();
  }

  public forEach(callback: (value: V) => void) {
    this.cache.forEach(callback);
  }

  public purgeStale(): boolean {
    return this.cache.purgeStale();
  }
}
