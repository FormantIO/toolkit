/* eslint-disable @typescript-eslint/ban-ts-comment */
// @ts-ignore
import PcdWorker from "./PcdLoaderWorker?worker&inline";
// @ts-ignore

import DataFetchWorker from "./DataFetchWorker?worker&inline";

export const PCD_WORKER_POOL_SIZE = 5;
export const DATA_FETCH_WORKER_POOL_SIZE = 10;

export class WorkerPoolService {
  static pcdWorkerPool: Worker[] = [];
  static dataFetchWorkerPool: Worker[] = [];

  static getPcdWorkerPool() {
    if (!Array.isArray(this.pcdWorkerPool) || !this.pcdWorkerPool.length) {
      for (let i = 0; i < PCD_WORKER_POOL_SIZE; i++) {
        const pcdWorker = new PcdWorker();
        this.pcdWorkerPool.push(pcdWorker);
      }
    }

    return this.pcdWorkerPool;
  }

  static getDataFetchWorkerPool() {
    if (
      !Array.isArray(this.dataFetchWorkerPool) ||
      !this.dataFetchWorkerPool.length
    ) {
      for (let i = 0; i < DATA_FETCH_WORKER_POOL_SIZE; i++) {
        const dataFetchWorker = new DataFetchWorker();
        this.dataFetchWorkerPool.push(dataFetchWorker);
      }
    }
    return this.dataFetchWorkerPool;
  }
}
