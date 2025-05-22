import { duration } from "./duration";

import { delay } from "./delay";
import { CacheOptions, LruCache } from "./LruCache";

export type PromiseCacheOptions<V> = CacheOptions<Promise<V>> & {
  expireRejectedPromiseValues?: boolean;
  rejectedPromiseValueTtl?: number;
};

export class PromiseLruCache<K, V> extends LruCache<K, Promise<V>> {
  protected expireRejectedPromiseValues: boolean;
  protected rejectedPromiseValueTtl: number;

  constructor(options: PromiseCacheOptions<V>) {
    super(options);

    this.expireRejectedPromiseValues =
      options.expireRejectedPromiseValues !== undefined
        ? options.expireRejectedPromiseValues
        : true;

    this.rejectedPromiseValueTtl =
      options.rejectedPromiseValueTtl !== undefined
        ? options.rejectedPromiseValueTtl
        : duration.second;

    if (this.rejectedPromiseValueTtl < 0) {
      throw new Error("rejectedPromiseValueTtl must not be negative");
    }
  }

  public set(key: K, value: Promise<V>, ttl?: number): void {
    super.set(key, value, ttl);

    if (this.expireRejectedPromiseValues) {
      value.catch(async () => {
        await delay(this.rejectedPromiseValueTtl);
        if (this.peek(key) === value) {
          this.delete(key);
        }
      });
    }
  }
}
