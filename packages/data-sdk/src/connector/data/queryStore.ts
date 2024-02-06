import {
  addMinutes,
  addSeconds,
  roundToNearestMinutes,
  startOfMinute,
} from "date-fns";
import { duration } from "../common/duration";
import { filterDataByType } from "../common/filterDataByType";
import { filterDataByTime } from "../common/filterDataByTime";
import { StoreCache } from "./StoreCache";
import { Fleet } from "../../Fleet";
import { IQuery } from "../../model/IQuery";
import { IStreamData } from "../../model/IStreamData";
import { IFilter } from "../../model/IFilter";
import { StreamType } from "../../model/StreamType";

function roundToNearestSecond(date: Date) {
  return new Date(Math.round(date.getTime() / 1000) * 1000);
}
export class QueryStore {
  private queryStoreCache = new StoreCache<
    IQuery,
    IStreamData[] | "too much data"
  >({
    capacity: 10_000,
    timeout: 20 * duration.second,
  });

  private liveQueryStoreCache = new StoreCache<
    IQuery,
    IStreamData[] | "too much data"
  >({
    capacity: 10_000,
    timeout: 200 * duration.millisecond,
  });

  public moduleQuery<T extends StreamType>(
    filter: IFilter,
    name: string,
    type: T,
    start: Date,
    end: Date,
    latestOnly: boolean = false
  ): IStreamData<T>[] | "too much data" | undefined {
    const q: IFilter = {
      ...filter,
      names: [name],
      types: [type],
    };
    const data = this.query(q, start, end, latestOnly);
    if (data === undefined || data === "too much data") {
      return data as any;
    }
    return filterDataByType(data, type);
  }

  public query(
    filter: IFilter,
    start: Date,
    end: Date,
    latestOnly: boolean = false
  ): IStreamData[] | "too much data" | undefined {
    const q: IQuery = {
      ...filter,
      start: startOfMinute(start).toISOString(),
      end: latestOnly
        ? addSeconds(roundToNearestSecond(end), 5).toISOString()
        : addMinutes(roundToNearestMinutes(end), 1).toISOString(),
      latestOnly,
    };
    const isLive = end > addSeconds(new Date(), -20);

    let data;
    if (isLive) {
      data = this.liveQueryCache(q);
    } else {
      data = this.queryCache(q);
    }

    if (!data || data === "too much data") {
      return data;
    }

    // return early because we might get data from near future that will be filtered out
    if (latestOnly) {
      return data;
    }

    return filterDataByTime(data, start, end);
  }

  private queryCache(
    query: IQuery
  ): IStreamData[] | "too much data" | undefined {
    return this.queryStoreCache.get(query, async () => {
      try {
        return await Fleet.queryTelemetry(query);
      } catch (error) {
        throw error;
      }
    });
  }

  private liveQueryCache(
    query: IQuery
  ): IStreamData[] | "too much data" | undefined {
    return this.liveQueryStoreCache.get(query, async () => {
      try {
        return await Fleet.queryTelemetry(query);
      } catch (error) {
        throw error;
      }
    });
  }
}
