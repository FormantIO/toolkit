import { SessionType } from "@formant/realtime-sdk/dist/protos/api/signaling/v1/signaling_pb";
import { Fleet } from "../../Fleet";
import { Device } from "../../devices/Device";
import { RealtimeListener, RealtimeMessage } from "../../devices/device.types";
import { IBitset } from "../../model/IBitset";
import { IJointState } from "../../model/IJointState";
import { ILocalization } from "../../model/ILocalization";
import { INumericSetEntry } from "../../model/INumericSetEntry";
import { IPointCloud } from "../../model/IPointCloud";
import { IStreamData } from "../../model/IStreamData";
import { ITransformNode } from "../../model/ITransformNode";
import { TelemetryResult } from "../../model/TelemetryResult";
import { defined } from "../common/defined";
import { IPose } from "../model/IPose";
import {
  CloseSubscription,
  IUniverseData,
  UniverseDataSource,
} from "../model/IUniverseData";
import { IUniverseGridMap } from "../model/IUniverseGridMap";
import { IUniverseOdometry } from "../model/IUniverseOdometry";
import { IUniversePath } from "../model/IUniversePath";
import { IUniversePointCloud } from "../model/IUniversePointCloud";

import { ILocation } from "../../model/ILocation";
import { IMarker3DArray } from "../../model/IMarker3DArray";
import {
  BasicUniverseDataConnector,
  DataResult,
} from "./BaseUniverseDataConnector";
import { IPcd } from "./pcd";

export class LiveUniverseData
  extends BasicUniverseDataConnector
  implements IUniverseData
{
  constructor() {
    super();
  }

  subscribeToPath(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: Symbol | IUniversePath) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }

  subscribeToImage(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (image: HTMLCanvasElement) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }

  subcribeToVideo(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (frame: HTMLVideoElement) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }

  subscribeToBitset(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IBitset | Symbol) => void
  ): CloseSubscription {
    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.bitset) {
          const bitsetData = msg.payload.bitset;
          const bitset: IBitset = {
            keys: [],
            values: [],
          };
          bitsetData.bits.forEach((bit: { key: string; value: boolean }) => {
            bitset.keys.push(bit.key);
            bitset.values.push(bit.value);
          });
          callback(bitset);
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }

    throw new Error("Telemetry bitset not implemented");
  }

  subscribeToOdometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniverseOdometry) => void,
    _trail?: number
  ): CloseSubscription {
    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.odometry) {
          const odometryData = msg.payload.odometry;
          callback({
            worldToLocal: odometryData.worldToLocal,
            pose: {
              translation: {
                x: odometryData.pose.translation.x,
                y: odometryData.pose.translation.y,
                z: odometryData.pose.translation.z,
              },
              rotation: {
                x: odometryData.pose.rotation.x,
                y: odometryData.pose.rotation.y,
                z: odometryData.pose.rotation.z,
                w: odometryData.pose.rotation.w,
              },
            },
            covariance: [],
          });
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    if (source.sourceType === "telemetry") {
      return this.addRemovableTelemetrySubscription<IUniverseOdometry>(
        deviceId,
        source,
        async (data: IStreamData[]): Promise<DataResult<IUniverseOdometry>> => {
          let foundLocalization: ILocalization | undefined;
          let odomValue: IUniverseOdometry | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "localization"
            ) {
              const [__, value] = _.points[_.points.length - 1];
              foundLocalization = value as ILocalization;
            }
          }
          if (foundLocalization && foundLocalization.odometry) {
            odomValue = {
              worldToLocal: foundLocalization.odometry.worldToLocal,
              pose: {
                translation: {
                  x: foundLocalization.odometry.pose.translation.x,
                  y: foundLocalization.odometry.pose.translation.y,
                  z: foundLocalization.odometry.pose.translation.z,
                },
                rotation: {
                  x: foundLocalization.odometry.pose.rotation.x,
                  y: foundLocalization.odometry.pose.rotation.y,
                  z: foundLocalization.odometry.pose.rotation.z,
                  w: foundLocalization.odometry.pose.rotation.w,
                },
              },
              covariance: [],
            };
          }
          return {
            deviceId,
            sourceId: source.id,
            data: odomValue,
          };
        },
        callback
      );
    }
    throw new Error("unexpected");
  }

  subscribeToPose(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IPose) => void
  ): CloseSubscription {
    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.pose) {
          callback(msg.payload.pose);
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    throw new Error("unexpected");
  }

  subscribeToNumeric(
    deviceId: string,
    source: UniverseDataSource,
    callback: (num: [number, number][]) => void
  ): CloseSubscription {
    if (source.sourceType === "realtime") {
      const listener = async (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.numeric) {
          callback([[Date.now(), msg.payload.numeric.value]]);
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    return () => {};
  }

  subscribeToNumericSet(
    deviceId: string,
    source: UniverseDataSource,
    callback: (entry: [number, INumericSetEntry[]][]) => void
  ): CloseSubscription {
    if (source.sourceType === "telemetry") {
      let device: Device | undefined;
      const intervalHandle = setInterval(async () => {
        if (!device) {
          device = await Fleet.getDevice(deviceId);
        }
        const now = new Date();
        const fifteenSecondsAgo = new Date(now.getTime() - 15 * 1000);
        const data: TelemetryResult[] = await device.getTelemetry(
          source.streamName,
          fifteenSecondsAgo,
          now
        );
        if (data.length > 0) {
          const numericsData = data[0].points as [number, INumericSetEntry[]][];
          callback(numericsData);
        }
      }, 1000);
      return () => {
        clearInterval(intervalHandle);
      };
    }
    return () => {};
  }

  async subscribeToRealtimeMessages(
    deviceId: string,
    streamName: string,
    callback: RealtimeListener
  ) {
    await this.createRealtimeConnection(deviceId, SessionType.OBSERVE);
    const device = this.mapRealtimeConnections.get(deviceId);
    if (device && device !== "loading") {
      device.startListeningToRealtimeDataStream({
        name: streamName,
      });
      device.addRealtimeListener((peerId, msg) => {
        if (msg.header.stream.streamName === streamName) {
          callback(peerId, msg);
        }
      });
    }
  }

  async unsubscribeToRealtimeMessages(
    deviceId: string,
    streamName: string,
    callback: RealtimeListener
  ) {
    const device = this.mapRealtimeConnections.get(deviceId);
    if (device && device !== "loading") {
      device.stopListeningToRealtimeDataStream({
        name: streamName,
      });
      device.removeRealtimeListener(callback);
    }
  }

  subscribeToJson<T>(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: T) => void
  ): CloseSubscription {
    if (source.sourceType === "telemetry") {
      return this.addRemovableTelemetrySubscription<T>(
        deviceId,
        source,
        async (data: IStreamData[]): Promise<DataResult<T>> => {
          let foundUrl: string | undefined;
          let jsonValue: T | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "json"
            ) {
              const [_time, value] = _.points[_.points.length - 1];
              foundUrl = value as string;
            }
          }
          if (foundUrl) {
            jsonValue = await fetch(foundUrl).then(
              (r) => r.json() as Promise<T>
            );
          }
          return {
            deviceId,
            sourceId: source.id,
            data: jsonValue,
          };
        },
        callback
      );
    } else if (source.sourceType === "realtime") {
      const listener = async (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.jsonString) {
          callback(JSON.parse(msg.payload.jsonString.value));
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    return () => {};
  }

  subscribeToText(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: string) => void
  ): CloseSubscription {
    if (source.sourceType === "telemetry") {
      return this.addRemovableTelemetrySubscription<string>(
        deviceId,
        source,
        async (data: IStreamData[]): Promise<DataResult<string>> => {
          let foundUrl: string | undefined;
          let textValue: string | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "text"
            ) {
              const [_time, value] = _.points[_.points.length - 1];
              foundUrl = value as string;
            }
          }
          if (foundUrl) {
            textValue = await fetch(foundUrl).then((r) => r.text());
          }
          return {
            deviceId,
            sourceId: source.id,
            data: textValue,
          };
        },
        callback
      );
    }
    return () => {};
  }

  subscribeToPointCloud(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniversePointCloud) => void
  ): () => void {
    const pcdWorker = this.getAvailablePCDWorker();
    if (!pcdWorker) {
      throw new Error("No available pointcloud worker");
    }
    if (
      source.sourceType === "telemetry" &&
      source.streamType !== "localization"
    ) {
      return this.addRemovableTelemetrySubscription(
        deviceId,
        source,
        async (
          data: IStreamData[]
        ): Promise<DataResult<IUniversePointCloud>> => {
          let found: IPointCloud | undefined;
          let pcd: IPcd | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "point cloud"
            ) {
              const [_time, pointCloud] = _.points[_.points.length - 1] as [
                number,
                IPointCloud
              ];
              found = pointCloud;
              break;
            }
          }
          if (found) {
            const { url } = found;
            pcd = await new Promise((resolve) => {
              pcdWorker.postMessage({ url });
              pcdWorker.onmessage = (
                ev: MessageEvent<{ url: string; pcd: IPcd }>
              ) => {
                if (ev.data.url === url) {
                  resolve(ev.data.pcd);
                }
              };
            });
          }
          return {
            deviceId,
            sourceId: source.id,
            data: {
              worldToLocal: found?.worldToLocal,
              pcd,
            },
          };
        },
        callback
      );
    }
    if (
      source.sourceType === "telemetry" &&
      source.streamType === "localization"
    ) {
      return this.addRemovableTelemetrySubscription(
        deviceId,
        source,
        async (
          data: IStreamData[]
        ): Promise<DataResult<IUniversePointCloud>> => {
          let found: ILocalization | undefined;
          let pcd: IPcd | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "localization"
            ) {
              const [_time, value] = _.points[_.points.length - 1] as [
                number,
                ILocalization
              ];
              found = value;
              break;
            }
          }
          if (found && found.pointClouds) {
            const { url } = found.pointClouds[0];
            pcd = await new Promise((resolve) => {
              pcdWorker.postMessage({ url });
              pcdWorker.onmessage = (
                ev: MessageEvent<{ url: string; pcd: IPcd }>
              ) => {
                if (ev.data.url === url) {
                  resolve(ev.data.pcd);
                }
              };
            });
          }
          return {
            deviceId,
            sourceId: source.id,
            data: {
              worldToLocal:
                found && found?.pointClouds && found?.pointClouds[0]
                  ? found.pointClouds[0].worldToLocal
                  : undefined,
              pcd,
            },
          };
        },
        callback
      );
    }
    if (source.sourceType === "realtime") {
      const listener = async (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.pointCloud) {
          const id = Math.random();
          const pcd = await new Promise<IPcd>((resolve) => {
            pcdWorker.postMessage({
              id,
              pointCloud: defined(msg.payload.pointCloud).data,
            });
            pcdWorker.onmessage = (
              ev: MessageEvent<{ id: number; pcd: IPcd }>
            ) => {
              if (ev.data.id === id) {
                resolve(ev.data.pcd);
              }
            };
          });
          callback({
            worldToLocal: msg.payload.pointCloud?.world_to_local,
            pcd,
          });
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    return () => {};
  }

  subscribeToGeometry(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IMarker3DArray) => void
  ): () => void {
    throw new Error("type error in realtime sdk");
    /*if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.markerArray) {
          callback(msg.payload.markerArray);
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    return () => {};*/
  }

  subscribeToJointState(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IJointState) => void
  ): () => void {
    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.jointState) {
          callback(msg.payload.jointState);
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }
    return () => {};
  }

  subscribeToGridMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniverseGridMap) => void
  ): () => void {
    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.jsonString) {
          const parsedMsg = JSON.parse(msg.payload.jsonString.value);
          callback({
            width: parsedMsg.info.width,
            height: parsedMsg.info.height,
            worldToLocal: {
              translation: { x: 0, y: 0, z: 0 },
              rotation: { x: 0, y: 0, z: 0, w: 1 },
            },
            resolution: parsedMsg.info.resolution,
            origin: {
              translation: parsedMsg.info.origin.position,
              rotation: parsedMsg.info.origin.orientation,
            },
            data: parsedMsg.data,
            alpha: parsedMsg.data.map(() => 255),
          });
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    } else if (
      source.sourceType === "telemetry" &&
      source.streamType === "localization"
    ) {
      return this.addRemovableTelemetrySubscription<IUniverseGridMap>(
        deviceId,
        source,
        async (data: IStreamData[]): Promise<DataResult<IUniverseGridMap>> => {
          let foundLocalization: ILocalization | undefined;
          let gridValue: IUniverseGridMap | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "localization"
            ) {
              const [_time, value] = _.points[_.points.length - 1];
              foundLocalization = value as ILocalization;
            }
          }
          if (foundLocalization && foundLocalization.map) {
            const canvas = document.createElement("canvas");
            const image = await this.fetchImage(foundLocalization.map.url);
            canvas.width = image.width;
            canvas.height = image.height;
            const ctx = canvas.getContext("2d", {
              willReadFrequently: true,
            });
            if (ctx) {
              ctx.drawImage(image, 0, 0);
            }
            const pixelData = ctx?.getImageData(
              0,
              0,
              image.width,
              image.height
            );
            const mapData: number[] = [];
            const alphaData: number[] = [];
            if (pixelData) {
              for (let i = 0; i < pixelData.data.length; i += 4) {
                const r = pixelData.data[i];
                const a = pixelData.data[i + 3];
                mapData.push(r);
                alphaData.push(a);
              }
            }
            gridValue = {
              width: foundLocalization.map.width,
              height: foundLocalization.map.height,
              worldToLocal: foundLocalization.map.worldToLocal,
              resolution: foundLocalization.map.resolution,
              origin: foundLocalization.map.origin,
              alpha: alphaData,
              data: mapData,
            };
          }
          return {
            deviceId,
            sourceId: source.id,
            data: gridValue,
          };
        },
        callback
      );
    }
    throw new Error("Not implemented");
  }

  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (frame: HTMLCanvasElement) => void
  ): () => void {
    /* if (
      source.sourceType === "hardware" &&
      source.rtcStreamName === "udp.127.0.0.1.5000"
    ) {
      const image = new Image();
      image.crossOrigin = "Anonymous";
      image.addEventListener(
        "load",
        () => {
          const canvas = document.createElement("canvas");
          canvas.width = image.width;
          canvas.height = image.height;
          const context = canvas.getContext("2d");
          if (context) {
            context.drawImage(image, 0, 0);
            callback(canvas);
          }
        },
        false
      );
      image.src =
        "https://threejs.org/examples/textures/2294472375_24a3b8ef46_o.jpg";
      return () => {};
    }*/

    const drawer = this.createH264Drawer();
    const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
    canvas.width = 0;
    canvas.height = 0;
    drawer.setCanvas(canvas);
    drawer.start();

    if (source.sourceType === "realtime") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.h264VideoFrame) {
          drawer.receiveEncodedFrame(msg.payload.h264VideoFrame);
          if (
            drawer &&
            drawer.canvas &&
            drawer.canvas.width > 0 &&
            drawer.canvas.height > 0
          ) {
            callback(drawer.canvas);
          }
        }
      };
      this.subscribeToRealtimeMessages(deviceId, source.rosTopicName, listener);
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rosTopicName,
          listener
        );
      };
    }

    if (source.sourceType === "hardware") {
      const listener = (_peerId: string, msg: RealtimeMessage) => {
        if (msg.payload.h264VideoFrame) {
          drawer.receiveEncodedFrame(msg.payload.h264VideoFrame);
          if (
            drawer &&
            drawer.canvas &&
            drawer.canvas.width > 0 &&
            drawer.canvas.height > 0
          ) {
            callback(drawer.canvas);
          }
        }
      };
      this.subscribeToRealtimeMessages(
        deviceId,
        source.rtcStreamName,
        listener
      );
      return () => {
        this.unsubscribeToRealtimeMessages(
          deviceId,
          source.rtcStreamName,
          listener
        );
      };
    }

    return () => {
      drawer.stop();
    };
  }

  subscribeToTransformTree(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void {
    if (source.sourceType === "telemetry") {
      return this.addRemovableTelemetrySubscription<ITransformNode>(
        deviceId,
        source,
        async (data: IStreamData[]): Promise<DataResult<ITransformNode>> => {
          let tfValue: ITransformNode | undefined;
          for (let i = 0; i < data.length; i += 1) {
            const _ = data[i];
            if (
              _.deviceId === deviceId &&
              _.name === source.streamName &&
              _.type === "transform tree"
            ) {
              const [__, value] = _.points[_.points.length - 1];
              tfValue = value as ITransformNode;
            }
          }
          return {
            deviceId,
            sourceId: source.id,
            data: tfValue,
          };
        },
        callback
      );
    }
    throw new Error(
      "not implemented for this source type " + source.sourceType
    );
  }

  subscribeToLocation(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: ILocation) => void
  ): () => void {
    throw new Error("Not implemented");
  }
}
