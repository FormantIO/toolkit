import { IDataPoint, IPointCloud, StreamType } from "@formant/data-sdk";
import {
  CloseSubscription,
  IPose,
  IUniverseData,
  IUniverseGridMap,
  IUniverseOdometry,
  NoData,
  UniverseDataSource,
} from "../main";
import { IJointState } from "@formant/data-sdk";
import { ILocation } from "@formant/data-sdk";
import { IMarker3DArray } from "@formant/data-sdk";
import { INumericSetEntry } from "@formant/data-sdk";
import { ITransformNode } from "@formant/data-sdk";
import { IPcd } from "../model/IPcd";
import { IUniversePath } from "../model/IUniversePath";
import { IUniversePointCloud } from "../model/IUniversePointCloud";
import { addSeconds, addYears } from "date-fns";
import { fork } from "../common/fork";
import { BasicUniverseDataConnector } from "./BaseUniverseDataConnector";
import { QueryStore } from "./queryStore";
import { StoreCache } from "./StoreCache";
import { IBitset } from "../../../data-sdk/src/model/IBitset";

const queryStore = new QueryStore();
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
    return this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        const dp = d[d.length - 1][1];

        let latestLocalization = dp;
        if (dp.url) {
          const asset = await fetch(dp.url);
          latestLocalization = await asset.json();
        }

        if (latestLocalization.path) {
          callback({
            worldToLocal: latestLocalization.path.worldToLocal,
            poses: latestLocalization.path.poses,
          });
        }
      }
    );
  }

  onTimeChange(time: Date | "live") {
    if (time === "live") {
      this.liveIntervalHandle = setInterval(() => {
        this.findDataForTime(new Date());
      }, 300);
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
    t: (f: "too much data" | IDataPoint<T>[] | undefined) => Promise<void>,
    deviceId: string,
    name: string,
    dataType: T,
    latestOnly: boolean
  ): (time: Date) => void {
    const fn = (time: Date) => {
      let start;
      if (latestOnly) {
        // lets look back a year if we are only looking for the latest data point
        start = addYears(time, -1);
      } else {
        start = addSeconds(time, -15);
      }
      let end;
      if (latestOnly) {
        end = time;
      } else {
        end = addSeconds(time, 5);
      }
      let data = queryStore.moduleQuery(
        {
          deviceIds: [deviceId],
        },
        name,
        dataType,
        start,
        end,
        latestOnly
      );
      if (data === undefined) {
        fork(t(undefined));
      } else if (data === "too much data") {
        fork(t("too much data"));
      } else {
        if (data.length > 0) {
          const streamData = data[0];
          const points = streamData.points;
          if (points.length > 0) {
            const lastPoint = points[points.length - 1];
            if (latestOnly) {
              let nearestPointTime = lastPoint[0];
              let nearestPoint = lastPoint[1];
              points.forEach((p) => {
                const pointTime = p[0];
                const point = p[1];
                if (
                  Math.abs(pointTime - time.getTime()) <
                  Math.abs(nearestPointTime - time.getTime())
                ) {
                  nearestPointTime = pointTime;
                  nearestPoint = point;
                }
              });
              fork(t([[nearestPointTime, nearestPoint]]));
            } else {
              fork(t(points));
            }
          } else {
            fork(t(undefined));
          }
        } else {
          fork(t(undefined));
        }
      }
    };
    this.timeFinders.push(fn);
    return fn;
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

    const jsonUnsubscribe = this.subscribeToJson<IUniversePointCloud>(
      deviceId,
      source,
      callback
    );
    const pointCloudUnsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "point cloud",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        const latestPointCloud = d[d.length - 1][1] as "string" | IPointCloud;
        if (typeof latestPointCloud === "string") {
          callback(JSON.parse(latestPointCloud) as IUniversePointCloud);
        } else {
          const { url } = latestPointCloud;
          this.pcdWorker.postMessage({ url });
          this.pcdWorker.onmessage = (
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
    const localizationUnsubscribe = this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        let latestLocalization = d[d.length - 1][1];
        if (latestLocalization.url) {
          const response = await fetch(latestLocalization.url);
          latestLocalization = await response.json();
        }
        if (latestLocalization.pointClouds) {
          const { url, worldToLocal } = latestLocalization.pointClouds[0];
          this.pcdWorker.postMessage({ url });
          this.pcdWorker.onmessage = (
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
      }
    );
    return () => {
      jsonUnsubscribe();
      pointCloudUnsubscribe();
      localizationUnsubscribe();
    };
  }
  subscribeToOdometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: Symbol | IUniverseOdometry) => void
  ): CloseSubscription {
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        const dp = d[d.length - 1][1];

        let latestLocalization = dp;
        if (dp.url) {
          const asset = await fetch(dp.url);
          latestLocalization = await asset.json();
        }

        if (latestLocalization.odometry) {
          callback({
            worldToLocal: latestLocalization.odometry.worldToLocal,
            pose: {
              translation: {
                x: latestLocalization.odometry.pose.translation.x,
                y: latestLocalization.odometry.pose.translation.y,
                z: latestLocalization.odometry.pose.translation.z,
              },
              rotation: {
                x: latestLocalization.odometry.pose.rotation.x,
                y: latestLocalization.odometry.pose.rotation.y,
                z: latestLocalization.odometry.pose.rotation.z,
                w: latestLocalization.odometry.pose.rotation.w,
              },
            },
            covariance: [],
          });
        }
      }
    );
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
      return this.subscribeTelemetry(deviceId, source, "json", async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }
        let jsonString = d[d.length - 1][1];
        if (jsonString.startsWith("http")) {
          const asset = await fetch(jsonString);
          jsonString = await asset.text();
        }
        callback(JSON.parse(jsonString) as IMarker3DArray);
      });
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
    if (source.sourceType !== "telemetry") {
      throw new Error("Telemetry sources only supported");
    }
    return this.subscribeTelemetry(
      deviceId,
      source,
      "localization",
      async (d) => {
        if (d === "too much data" || d === undefined) {
          callback(NoData);
          return;
        }

        const dp = d[d.length - 1][1];

        let latestLocalization = dp;
        if (dp.url) {
          const asset = await fetch(dp.url);
          latestLocalization = await asset.json();
        }

        if (latestLocalization.map) {
          const canvas = document.createElement("canvas");
          const image = await this.fetchImage(latestLocalization.map.url);
          canvas.width = image.width;
          canvas.height = image.height;
          const ctx = canvas.getContext("2d");
          if (ctx) {
            ctx.drawImage(image, 0, 0);
          }
          const pixelData = ctx?.getImageData(0, 0, image.width, image.height);
          const mapData: number[] = [];
          if (pixelData) {
            for (let i = 0; i < pixelData.data.length; i += 4) {
              const r = pixelData.data[i];
              mapData.push(r);
            }
          }
          const gridValue = {
            width: latestLocalization.map.width,
            height: latestLocalization.map.height,
            worldToLocal: latestLocalization.map.worldToLocal,
            resolution: latestLocalization.map.resolution,
            origin: latestLocalization.map.origin,
            canvas,
            data: mapData,
          };
          callback(gridValue);
        }
      }
    );
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

      const currentVideo = d[d.length - 1][1];
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
        callback(d[d.length - 1][1]);
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
        const latestPosition = d[d.length - 1][1];
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
      let jsonString = d[d.length - 1][1];
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
      callback(d[d.length - 1][1]);
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
      const currentImageUrl = d[d.length - 1][1].url;
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
