// @ts-ignore
import DataLoaderWorker from "./data-loader.worker?worker&inline";

import { PromiseLruCache } from "../../common/PromiseLruCache";
import { range } from "../../common/range";

const workerCount = 4;

type JsonPrimitive = string | number | boolean | null;

interface IJsonMap {
  [member: string]: JsonPrimitive | IJsonArray | IJsonMap;
}

// eslint-disable-next-line @typescript-eslint/no-empty-object-type
interface IJsonArray extends Array<JsonPrimitive | IJsonArray | IJsonMap> {}

export type Json = IJsonMap | IJsonArray | JsonPrimitive;

export interface IDataLoaderResult {
  json: Json;
  preview: string;
  length: number;
}

export class DataLoader {
  private static references: number = 0;
  private static instance: DataLoader | null = null;

  public static get(): DataLoader {
    DataLoader.references++;
    if (DataLoader.instance) {
      return DataLoader.instance;
    }
    DataLoader.instance = new DataLoader();
    return DataLoader.instance;
  }

  public static release(loader: DataLoader) {
    if (DataLoader.instance !== loader) {
      console.error("Releasing an unallocated/unowned loader!");
      return;
    }

    DataLoader.references--;
    if (DataLoader.references > 0) {
      return;
    }

    DataLoader.instance.workers?.forEach((_) => _.terminate());
    DataLoader.instance.workers = undefined;
    DataLoader.instance.cache.clear();
    DataLoader.instance = null;
    DataLoader.references = 0;
  }

  private workers: Worker[] | undefined;
  private cache = new PromiseLruCache<string, IDataLoaderResult>({
    name: "DataLoader-cache",
    max: 5000,
  });
  private nextWorkerIndex: number = -1;
  private resolveMap: {
    [url: string]: (
      value: IDataLoaderResult | PromiseLike<IDataLoaderResult>
    ) => void;
  } = {};
  private rejectMap: {
    [url: string]: (reason?: unknown) => void;
  } = {};

  private constructor() {}

  public load(url: string): Promise<IDataLoaderResult> {
    const cached = this.cache.get(url);
    if (cached) {
      return cached;
    }

    const worker = this.nextWorker();
    const loaded = new Promise<IDataLoaderResult>((resolve, reject) => {
      this.resolveMap[url] = resolve;
      this.rejectMap[url] = reject;
    });
    this.cache.set(url, loaded);
    worker.postMessage(url);
    return loaded;
  }

  private nextWorker() {
    if (!this.workers) {
      this.workers = range(0, workerCount).map((_) => {
        const worker = new DataLoaderWorker();
        // eslint-disable-next-line @typescript-eslint/no-explicit-any
        worker.onmessage = ({ data }: any) => {
          const { url } = data;
          if (!url) {
            return;
          }
          const reject = this.rejectMap[url];
          const resolve = this.resolveMap[url];
          delete this.rejectMap[url];
          delete this.resolveMap[url];
          if (!reject || !resolve) {
            return;
          }
          if (data.error) {
            const error = new Error(`Worker failure: ${data.error}`);
            (error as Error).cause = data.error;
            reject(error);
            return;
          }
          resolve({
            json: data.json,
            preview: data.preview,
            length: data.length,
          });
        };
        return worker;
      });
    }
    this.nextWorkerIndex = (this.nextWorkerIndex + 1) % workerCount;
    return this.workers[this.nextWorkerIndex];
  }
}
