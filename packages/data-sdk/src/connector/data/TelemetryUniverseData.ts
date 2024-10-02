import { addMilliseconds, addSeconds, addYears } from "date-fns";
import { IBitset } from "../../model/IBitset";
import { IDataPoint } from "../../model/IDataPoint";
import { IImage } from "../../model/IImage";
import { IJointState } from "../../model/IJointState";
import { ILocalization } from "../../model/ILocalization";
import { ILocation } from "../../model/ILocation";
import { IMarker3DArray } from "../../model/IMarker3DArray";
import { INumericSetEntry } from "../../model/INumericSetEntry";
import { IPointCloud } from "../../model/IPointCloud";
import { ITransform } from "../../model/ITransform";
import { ITransformNode } from "../../model/ITransformNode";
import { IVideo } from "../../model/IVideo";
import { StreamType } from "../../model/StreamType";
import { fork } from "../common/fork";
import { IPose } from "../model/IPose";
import {
  CloseSubscription,
  IUniverseData,
  NoData,
  UniverseDataSource,
} from "../model/IUniverseData";
import { IUniverseGridMap } from "../model/IUniverseGridMap";
import { IUniverseOdometry } from "../model/IUniverseOdometry";
import { IUniversePath } from "../model/IUniversePath";
import { IUniversePointCloud } from "../model/IUniversePointCloud";
import { BasicUniverseDataConnector } from "./BaseUniverseDataConnector";
// @ts-ignore
import DataFetchWorker from "./DataFetchWorker?worker&inline";
import { IPcd } from "./pcd";
// @ts-ignore
import PCDLoaderWorker from "./PcdLoaderWorker?worker&inline";
import { StoreCache } from "./StoreCache";

export class TelemetryUniverseData
  extends BasicUniverseDataConnector
  implements IUniverseData
{
  liveIntervalHandle: number | undefined;
  constructor() {
    super();
    this.timeChangeListeners.push(this.onTimeChange.bind(this));
    this.onTimeChange("live");
  }

  subscribeToBitset(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IBitset | Symbol) => void
  ): CloseSubscription {
    throw new Error("Method not implemented for telemetry universe connector.");
  }

  subscribeToPath(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IUniversePath) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    const dataFetchWorker = new DataFetchWorker();
    let latestTimestamp = 0;

    const unsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (data) => {
        if (data === "too much data" || data === undefined) {
          callback(NoData);
          return;
        }
        const nearestPoint = this.getNearestPoint(data);
        const datapoint = nearestPoint[1] as ILocalization;
        const timestamp = nearestPoint[0];

        if (timestamp === latestTimestamp) {
          return;
        }
        latestTimestamp = timestamp;

        if (datapoint.url) {
          const response = (await fetch(datapoint.url).then((res) =>
            res.json()
          )) as ILocalization;
          if (response.path && timestamp >= latestTimestamp) {
            callback(response.path);
          }
          return;
        } else if (datapoint.path && timestamp >= latestTimestamp) {
          callback(datapoint.path);
          return;
        }
      }
    );

    return () => {
      dataFetchWorker.terminate();
      unsubscribe();
    };
  }

  onTimeChange(time: Date | "live") {
    if (time === "live") {
      this.liveIntervalHandle = setInterval(() => {
        this.findDataForTime(new Date());
      }, 300) as unknown as number;
    } else {
      if (this.liveIntervalHandle) {
        clearInterval(this.liveIntervalHandle);
      }
      this.findDataForTime(time);
    }
  }

  findDataForTime(time: Date) {
    this.timeFinders.forEach((finder) => {
      finder(time);
    });
  }

  timeFinders: ((time: Date) => void)[] = [];

  addFinder<T extends StreamType>(
    callback: (
      f: "too much data" | IDataPoint<T>[] | undefined
    ) => Promise<void>,
    deviceId: string,
    name: string,
    dataType: T,
    latestOnly: boolean
  ): (time: Date) => void {
    const fn = (time: Date) => {
      // for latestonly we look back a year until now
      // otherwise look 60 seconds back and 5 seconds forward
      const start = latestOnly ? addYears(time, -1) : addSeconds(time, -60);
      const end = latestOnly ? addMilliseconds(time, 1) : addSeconds(time, 5);

      const data = this.queryStore.moduleQuery(
        { deviceIds: [deviceId] },
        name,
        dataType,
        start,
        end,
        latestOnly
      );

      if (data === undefined) {
        fork(callback(undefined));
        return;
      }
      if (data === "too much data") {
        fork(callback("too much data"));
        return;
      }
      if (data.length === 0) {
        fork(callback(undefined));
        return;
      }

      // streamData can have multiple items for a different set of tags
      // accumulate all the points into a single array
      const points = data.reduce((acc: IDataPoint<T>[], d) => {
        return acc.concat(d.points);
      }, []);

      if (!points || points.length === 0) {
        fork(callback(undefined));
        return;
      }

      // the query will contain a year of data, we need to filter it down to the relevant time
      if (latestOnly) {
        const lastPoint = points[points.length - 1];
        const lastTime = lastPoint[0];
        const filteredPoints = points.filter(
          (p) => p[0] > addSeconds(lastTime, -15).getTime()
        );
        fork(callback(filteredPoints));
        return;
      }

      fork(callback(points));
    };
    this.timeFinders.push(fn);
    return fn;
  }

  private getNearestPoint(
    points: IDataPoint<StreamType>[],
    time: Date | "live" = this.time
  ) {
    const _time =
      time === "live" ? addMilliseconds(new Date(), 1) : (this.time as Date);
    let nearestPointTime = points[0][0];
    let nearestPoint = points[0][1];
    points.forEach((p) => {
      const pointTime = p[0];
      const point = p[1];
      if (
        Math.abs(pointTime - _time.getTime()) <
        Math.abs(nearestPointTime - _time.getTime())
      ) {
        nearestPointTime = pointTime;
        nearestPoint = point;
      }
    });
    return [nearestPointTime, nearestPoint] as IDataPoint<StreamType>;
  }

  removeFinder(fn: (time: Date) => void) {
    this.timeFinders = this.timeFinders.filter((f) => f !== fn);
  }

  subscribeTelemetry<T extends StreamType>(
    deviceId: string,
    source: UniverseDataSource,
    type: T,
    callback: (
      f: "too much data" | IDataPoint<T>[] | undefined
    ) => Promise<void>
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Invalid source type");
    }
    const fn = this.addFinder(
      callback,
      deviceId,
      source.streamName,
      type,
      source.latestDataPoint || false
    );
    return () => {
      this.removeFinder(fn);
    };
  }

  subscribeToPointCloud(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniversePointCloud | Symbol) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    const pcdWorker = new PCDLoaderWorker();
    const dataFetchWorker = new DataFetchWorker();

    // Call the function and handle the resolved data type
    let pointCloudUnsubscribe = () => {};
    let localizationUnsubscribe = () => {};
    if (source.streamType === "point cloud") {
      pointCloudUnsubscribe = this.subscribeTelemetry(
        deviceId,
        source,
        "point cloud",
        async (d) => {
          if (d === "too much data" || d === undefined) {
            callback(NoData);
            return;
          }
          const latestPointCloud = this.getNearestPoint(d)[1] as IPointCloud;
          if (typeof latestPointCloud === "string") {
            callback(JSON.parse(latestPointCloud) as IUniversePointCloud);
          } else {
            const { url } = latestPointCloud;
            pcdWorker.postMessage({ url });
            pcdWorker.onmessage = (
              ev: MessageEvent<{ url: string; pcd: IPcd }>
            ) => {
              if (ev.data.url === url) {
                callback({
                  worldToLocal: latestPointCloud.worldToLocal,
                  pcd: ev.data.pcd,
                });
              }
            };
          }
        }
      );
    } else if (source.streamType === "localization") {
      localizationUnsubscribe = this.subscribeTelemetry(
        deviceId,
        source,
        "localization",
        async (d) => {
          if (d === "too much data" || d === undefined) {
            callback(NoData);
            return;
          }
          let latestLocalization = this.getNearestPoint(d)[1] as ILocalization;
          if (latestLocalization.url) {
            dataFetchWorker.postMessage({ url: latestLocalization.url });
            dataFetchWorker.onmessage = (
              ev: MessageEvent<{ url: string; response: any }>
            ) => {
              latestLocalization = ev.data.response;

              if (latestLocalization.pointClouds) {
                const { url, worldToLocal } = latestLocalization.pointClouds[0];
                pcdWorker.postMessage({ url });
                pcdWorker.onmessage = (
                  ev: MessageEvent<{ url: string; pcd: IPcd }>
                ) => {
                  if (ev.data.url === url) {
                    callback({
                      worldToLocal,
                      pcd: ev.data.pcd,
                    });
                  }
                };
              }
            };
          }
        }
      );
    }

    return () => {
      pcdWorker.terminate();
      dataFetchWorker.terminate();
      pointCloudUnsubscribe();
      localizationUnsubscribe();
    };
  }

  subscribeToOdometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IUniverseOdometry) => void,
    trail: number = 0 // how many seconds of previous odometry points to include in the trail
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    const dataFetchWorker = new DataFetchWorker();
    const unsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (data) => {
        if (data === "too much data" || data === undefined) {
          callback(NoData);
          return;
        }
        const currentDatapoint = this.getNearestPoint(
          data
        ) as IDataPoint<"localization">;

        let odometry: ILocalization["odometry"];
        if (currentDatapoint[1].url) {
          try {
            const response = await fetch(currentDatapoint[1].url);
            const jsonResponse = (await response.json()) as ILocalization;
            odometry = jsonResponse.odometry;
          } catch (error) {
            console.error("Failed to fetch odometry data:", error);
            throw error;
          }
        } else {
          odometry = currentDatapoint[1].odometry;
        }

        if (trail) {
          const trailDatapoints = data.filter(
            (datapoint) =>
              datapoint[0] <= currentDatapoint[0] &&
              datapoint[0] >= currentDatapoint[0] - trail * 1000
          );

          const trailPromises = trailDatapoints.map(async (datapoint) => {
            if (datapoint[1].url) {
              try {
                const response = await fetch(datapoint[1].url);
                const jsonResponse = (await response.json()) as ILocalization;
                return [datapoint[0], jsonResponse.odometry?.pose] as [
                  number,
                  ITransform
                ];
              } catch (error) {
                console.error("Failed to fetch trail odometry data:", error);
                throw error;
              }
            }
            return [datapoint[0], datapoint[1].odometry?.pose] as [
              number,
              ITransform
            ];
          });

          try {
            const trailResults = await Promise.all(trailPromises);
            callback({
              worldToLocal: odometry!.worldToLocal,
              pose: odometry!.pose,
              trail: trailResults,
              covariance: [],
            });
            return;
          } catch (error) {
            console.error("Failed to process trail data:", error);
            throw error;
          }
        }

        callback({
          worldToLocal: odometry!.worldToLocal,
          pose: odometry!.pose,
          covariance: [],
        });
        return;
      }
    );

    return () => {
      dataFetchWorker.terminate();
      unsubscribe();
    };
  }

  subscribeToPose(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: Symbol | IPose) => void
  ): CloseSubscription {
    throw new Error("Method not implemented for telemetry universe connector.");
  }

  subscribeToGeometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IMarker3DArray) => void
  ): CloseSubscription {
    if (source.sourceType === "telemetry") {
      const dataFetchWorker = new DataFetchWorker();
      const unsubscribe = this.subscribeTelemetry(
        deviceId,
        source,
        "json",
        async (d) => {
          if (d === "too much data" || d === undefined) {
            callback(NoData);
            return;
          }
          let jsonString = this.getNearestPoint(d)[1] as string;
          if (jsonString.startsWith("http")) {
            dataFetchWorker.postMessage({ url: jsonString });
            dataFetchWorker.onmessage = (
              ev: MessageEvent<{ url: string; response: any }>
            ) => {
              jsonString = JSON.stringify(ev.data.response);
              callback(JSON.parse(jsonString) as IMarker3DArray);
            };
          } else {
            callback(JSON.parse(jsonString) as IMarker3DArray);
          }
        }
      );

      return () => {
        dataFetchWorker.terminate();
        unsubscribe();
      };
    } else {
      throw new Error("Realtime geometry note supported");
    }
  }

  subscribeToJointState(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IJointState) => void
  ): CloseSubscription {
    return this.subscribeToJson<IJointState>(deviceId, source, callback);
  }

  subscribeToGridMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IUniverseGridMap) => void
  ): CloseSubscription {
    const mapDataCache: { [url: string]: any } = {};

    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    const dataFetchWorker = new DataFetchWorker();
    const unsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }

        const dp = this.getNearestPoint(d)[1] as ILocalization;
        if (dp.url) {
          if (mapDataCache[dp.url]) {
            callback(mapDataCache[dp.url]);
            return;
          }

          dataFetchWorker.postMessage({ url: dp.url });

          dataFetchWorker.onmessage = async (ev: MessageEvent) => {
            const latestLocalization = ev.data.response.map;

            if (latestLocalization) {
              const gridValue = {
                width: latestLocalization.width,
                height: latestLocalization.height,
                worldToLocal: latestLocalization.worldToLocal,
                resolution: latestLocalization.resolution,
                origin: latestLocalization.origin,
                url: latestLocalization.url,
                //alpha: alphaData,
                //data: mapData,
              };
              mapDataCache[dp.url!] = JSON.parse(JSON.stringify(gridValue));
              callback(gridValue);
            }
          };
        }
      }
    );

    return () => {
      dataFetchWorker.terminate();
      unsubscribe();
    };
  }

  videoCache = new StoreCache<string, HTMLVideoElement>({
    capacity: 50,
  });

  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (frame: Symbol | HTMLCanvasElement) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(deviceId, source, "video", async (d) => {
      if (d === "too much data" || d === undefined) {
        callback(NoData);
        return;
      }

      const currentVideo = this.getNearestPoint(d)[1] as IVideo;
      const { url } = currentVideo;

      const video = this.videoCache.get(url, async () => {
        return new Promise((resolve) => {
          const videoEl = document.createElement("video");
          videoEl.src = url;
          videoEl.onload = () => {
            resolve;
          };
        });
      });
      if (video) {
        const canvas = document.createElement("canvas");
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        const ctx = canvas.getContext("2d");
        if (ctx) {
          ctx.drawImage(video, 0, 0);
        }
        callback(canvas);
      }
    });
  }
  subscribeToTransformTree(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | ITransformNode) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(
      deviceId,
      source,
      "transform tree",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        callback(this.getNearestPoint(d)[1] as ITransformNode);
      }
    );
  }

  subscribeToLocation(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | ILocation) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    const jsonUnsubscribe = this.subscribeToJson<ILocation>(
      deviceId,
      source,
      callback
    );
    const locationUnsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "location",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        const latestPosition = this.getNearestPoint(d)[1] as ILocation;
        callback(latestPosition);
      }
    );

    return () => {
      jsonUnsubscribe();
      locationUnsubscribe();
    };
  }

  subscribeToJson<T>(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | T) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(deviceId, source, "json", async (d) => {
      if (d === "too much data" || d === undefined) {
        callback(NoData);
        return;
      }
      let jsonString = this.getNearestPoint(d)[1] as string;
      if (jsonString.startsWith("http")) {
        const asset = await fetch(jsonString);
        jsonString = await asset.text();
      }
      callback(JSON.parse(jsonString));
    });
  }

  subscribeToText(
    deviceId: string,
    source: UniverseDataSource,
    callback: (text: string | Symbol) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(deviceId, source, "text", async (d) => {
      if (d === "too much data" || d === undefined) {
        callback(NoData);
        return;
      }
      callback(this.getNearestPoint(d)[1] as string);
    });
  }

  subscribeToNumeric(
    deviceId: string,
    source: UniverseDataSource,
    callback: (num: Symbol | [number, number][]) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(deviceId, source, "numeric", async (d) => {
      if (d === "too much data" || d === undefined) {
        callback(NoData);
        return;
      }
      callback(d);
    });
  }

  subscribeToNumericSet(
    deviceId: string,
    source: UniverseDataSource,
    callback: (entry: Symbol | [number, INumericSetEntry[]][]) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(
      deviceId,
      source,
      "numeric set",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        callback(d);
      }
    );
  }

  subscribeToImage(
    deviceId: string,
    source: UniverseDataSource,
    callback: (image: Symbol | HTMLCanvasElement) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(deviceId, source, "image", async (d) => {
      if (d === "too much data" || d === undefined) {
        callback(NoData);
        return;
      }
      const currentImageUrl = (this.getNearestPoint(d)[1] as IImage).url;
      const currentImage = new Image();
      currentImage.src = currentImageUrl;
      currentImage.onload = () => {
        const canvas = document.createElement("canvas");
        canvas.width = currentImage.width;
        canvas.height = currentImage.height;
        const ctx = canvas.getContext("2d");
        if (ctx) {
          ctx.drawImage(currentImage, 0, 0);
          callback(canvas);
        }
      };
    });
  }
}
