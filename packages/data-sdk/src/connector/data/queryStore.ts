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
    timeout: 1 * duration.second,
  });

  public moduleQuery<T extends StreamType>(
    filter: IFilter,
    name: string,
    type: T,
    start: Date,
    end: Date,
    latestOnly: boolean = false
  ): IStreamData<T>[] | "too much data" | undefined {
    const q: IQuery = {
      ...filter,
      names: [name],
      types: [type],
      start: start.toISOString(),
      end: end.toISOString(),
      latestOnly,
    };
    const data = this.query(q);
    if (data === undefined || data === "too much data") {
      return data as any;
    }
    return filterDataByType(data, type);
  }

  public query(q: IQuery): IStreamData[] | "too much data" | undefined {
    const isLive = new Date(q.end) > addSeconds(new Date(), -20);
    const start = startOfMinute(new Date(q.start)).toISOString();
    const end = q.latestOnly
      ? addSeconds(roundToNearestSecond(new Date(q.end)), 5).toISOString()
      : addMinutes(roundToNearestMinutes(new Date(q.end)), 1).toISOString();

    let data;
    if (isLive) {
      data = this.liveQueryCache({
        ...q,
        start,
        end,
      });
    } else {
      data = this.queryCache({
        ...q,
        start,
        end,
      });
    }

    if (!data || data === "too much data") {
      return data;
    }

    // return early because we might get data from near future that will be filtered out
    if (q.latestOnly) {
      return data;
    }

    return filterDataByTime(data, new Date(q.start), new Date(q.end));
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
