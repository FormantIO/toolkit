import {
  CloseSubscription,
  DataSourceState,
  IHardwareStream,
  InteractionContext,
  IPcd,
  ITelemetryRosStream,
  ITelemetryStream,
  IUniverseData,
  IUniverseStatistics,
  RealtimeButtonConfiguration,
  UniverseDataSource,
} from "@formant/universe";
import { IJointState } from "@formant/universe/dist/types/data-sdk/src/model/IJointState";
import { ILocation } from "@formant/universe/dist/types/data-sdk/src/model/ILocation";
import { IMap } from "@formant/universe/dist/types/data-sdk/src/model/IMap";
import { IMarker3DArray } from "@formant/universe/dist/types/data-sdk/src/model/IMarker3DArray";
import { INumericSetEntry } from "@formant/universe/dist/types/data-sdk/src/model/INumericSetEntry";
import { ITransform } from "@formant/universe/dist/types/model/ITransform";
import { ITransformNode } from "@formant/universe/dist/types/model/ITransformNode";
import { IGridMap } from "@formant/universe/dist/types/universe/src/model/IGridMap";

export const SPOT_ID = "abc";
export const ARM1_ID = "asdfadsfas";
export const ARM2_ID = "124fasd";
export const ARM3_ID = "77hrtesgdafdsh";

export class SimulatedUniverseData implements IUniverseData {
  sendRealtimePose(
    _deviceId: string,
    _streamName: string,
    _pose: ITransform
  ): void {
    throw new Error("Method not implemented.");
  }
  subscribeToNumeric(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (num: number) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }
  subscribeToNumericSet(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (entry: INumericSetEntry) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }
  getInteractionContext(): InteractionContext {
    throw new Error("Method not implemented.");
  }
  addInteractionContextChangedListener(
    _callback: (c: InteractionContext) => void
  ): void {
    throw new Error("Method not implemented.");
  }
  removeInteractionContextChangedListener(
    _callback: (c: InteractionContext) => void
  ): void {
    throw new Error("Method not implemented.");
  }
  getRealtimeButtons(
    _deviceId: string
  ): Promise<RealtimeButtonConfiguration[]> {
    throw new Error("Method not implemented.");
  }
  sendRealtimeButtonState(
    _deviceId: string,
    _streamName: string,
    _state: boolean
  ): void {
    throw new Error("Method not implemented.");
  }
  subscribeToGridMap(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IGridMap) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }
  subscribeToVideo(
    _deviceId: string,
    _source: UniverseDataSource,
    callback: (data: HTMLCanvasElement) => void
  ): () => void {
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
      "https://formant-3d-models.s3.us-west-2.amazonaws.com/stereo.jpg";
    return () => {};
  }
  subscribeToJson<T>(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: T) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }
  subscribeToText(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (text: string) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }
  getStatistics(): Promise<IUniverseStatistics> {
    throw new Error("Method not implemented.");
  }
  subscribeDataSourceStateChange(
    _deviceId: string,
    _source: UniverseDataSource,
    _onDataSourceStateChange?: (state: DataSourceState) => void
  ): CloseSubscription {
    throw new Error("Method not implemented.");
  }

  time = Date.now();
  setTime(time: number | "live"): void {
    if (time === "live") {
      throw new Error("Not implemented");
    } else {
      this.time = time;
    }
  }
  async getLatestTransformTrees(
    deviceId: string
  ): Promise<{ streamName: string; transformTree: ITransformNode }[]> {
    if (deviceId === SPOT_ID) {
      return [
        {
          streamName: "spotTf",
          transformTree: await fetch(
            "https://formant-3d-models.s3.us-west-2.amazonaws.com/spotjoint_sit.json"
          ).then((_) => _.json() as ITransformNode),
        },
      ];
    }
    return [];
  }

  async getLatestLocations(): Promise<
    {
      streamName: string;
      location: ILocation;
    }[]
  > {
    return [
      {
        streamName: "spotLocation",
        location: {
          latitude: 45.4661989,
          longitude: -122.5782375,
        },
      },
    ];
  }

  async getDeviceContexts(): Promise<
    { deviceName: string; deviceId: string }[]
  > {
    return [
      { deviceName: "Spot-9000", deviceId: SPOT_ID },
      { deviceName: "Roboarm 1", deviceId: ARM1_ID },
      { deviceName: "Roboarm 2", deviceId: ARM2_ID },
      { deviceName: "Roboarm 3", deviceId: ARM3_ID },
    ];
  }

  async getDeviceContextName(deviceId: string): Promise<string | undefined> {
    if (deviceId === SPOT_ID) {
      return "Spot-9000";
    }
    if (deviceId === ARM1_ID) {
      return "Roboarm 1";
    }
    if (deviceId === ARM2_ID) {
      return "Roboarm 2";
    }
    if (deviceId === ARM3_ID) {
      return "Roboarm 3";
    }
    return undefined;
  }

  async getTelemetryStreamType(
    _deviceId: string,
    streamName: string
  ): Promise<string | undefined> {
    if (streamName === "spotTf") {
      return "transform tree";
    }
    return undefined;
  }

  subscribeToPointCloud(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IPcd) => void
  ): () => void {
    return () => {};
  }

  subscribeToGeometry(
    _deviceId: string,
    _source: UniverseDataSource,
    callback: (data: IMarker3DArray) => void
  ): () => void {
    callback({
      markers: [...Array(100).keys()].map(() => ({
        id: Math.random(),
        ns: `cube${Math.random()}`,
        type: "cube",
        action: "add",
        lifetime: 100000,
        frame_id: "base_link",
        points: [],
        text: "",
        mesh_resource: "",
        frame_locked: false,
        mesh_use_embedded_materials: false,
        color: { r: 1, g: 1, b: 1, a: 0.4 },
        colors: [],
        pose: {
          translation: {
            x: 20 * Math.random() - 10,
            y: 20 * Math.random() - 10,
            z: 0,
          },
          rotation: {
            x: 0,
            y: 0,
            z: 0,
            w: 1,
          },
        },
        scale: {
          x: 0.1,
          y: 0.1,
          z: 0.1,
        },
      })),
    });
    return () => {};
  }

  subscribeToJointState(
    _deviceId: string,
    _source: UniverseDataSource,
    callback: (data: IJointState) => void
  ): () => void {
    if (_deviceId === SPOT_ID) {
      window.setInterval(() => {
        callback({
          name: [
            "fl.hx",
            "fl.hy",
            "fl.kn",
            "fr.hx",
            "fr.hy",
            "fr.kn",
            "hl.hx",
            "hl.hy",
            "hl.kn",
            "hr.hx",
            "hr.hy",
            "hr.kn",
          ],
          position: [
            0,
            Math.sin(this.time / 1000),
            Math.cos(this.time / 1000),

            0,
            Math.cos(this.time / 1000),
            Math.sin(this.time / 1000),

            0,
            Math.sin(this.time / 1000),
            Math.cos(this.time / 1000),

            0,
            Math.cos(this.time / 1000),
            Math.sin(this.time / 1000),
          ],
        });
      }, 60 / 12);
    } else if (_deviceId === ARM1_ID) {
      window.setInterval(() => {
        callback({
          name: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
          ],
          position: [
            Math.sin(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
          ],
        });
      }, 60 / 12);
    } else if (_deviceId === ARM2_ID) {
      window.setInterval(() => {
        callback({
          name: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
          ],
          position: [
            Math.sin(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.sin(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
          ],
        });
      }, 60 / 12);
    } else if (_deviceId === ARM3_ID) {
      window.setInterval(() => {
        callback({
          name: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
          ],
          position: [
            Math.sin(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
            Math.cos(this.time / 1000 / 3),
          ],
        });
      }, 60 / 12);
    }
    return () => {};
  }

  subscribeToMap(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IMap) => void
  ): () => void {
    return () => {};
  }

  async getTelemetryStreams(deviceId: string): Promise<ITelemetryStream[]> {
    if (deviceId === SPOT_ID) {
      return [
        {
          name: "spotTf",
          configuration: {
            type: "transform tree",
          },
        },
      ];
    }
    return [];
  }

  async getTeleopRosStreams(deviceId: string): Promise<ITelemetryRosStream[]> {
    if (deviceId === SPOT_ID) {
      return [
        {
          topicType: "sensor_msgs/JointState",
          topicName: "spotJoints",
        },
        {
          topicType: "visualization_msgs/MarkerArray",
          topicName: "spotMarkers",
        },
      ];
    }
    return [
      {
        topicType: "sensor_msgs/JointState",
        topicName: "armJoints",
      },
    ];
  }

  async getUrdfs(deviceId: string): Promise<string[]> {
    if (deviceId === SPOT_ID) {
      return ["https://formant-3d-models.s3.us-west-2.amazonaws.com/spot.zip"];
    }
    return ["https://formant-3d-models.s3.us-west-2.amazonaws.com/arm.zip"];
  }

  async getHardwareStreams(_deviceId: string): Promise<IHardwareStream[]> {
    return [];
  }

  subscribeToTransformTree(
    deviceId: string,
    _source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void {
    if (deviceId === SPOT_ID) {
      callback({
        url: "https://formant-3d-models.s3.us-west-2.amazonaws.com/spotjoint_sit.json",
      });
    }
    return () => {};
  }

  subscribeToLocation(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ILocation) => void
  ): () => void {
    if (source.sourceType === "telemetry") {
      const delta = 0.00001;
      if (deviceId === SPOT_ID) {
        window.setInterval(() => {
          const newValue =
            -122.5782375 + 2 * delta * Math.sin(this.time / 1000 / 3);
          callback({
            latitude: 45.4661989,
            longitude: newValue,
          });
        }, 60 / 12);
      } else if (deviceId === ARM1_ID) {
        callback({
          latitude: 45.4661989 + delta,
          longitude: -122.5782375,
        });
      } else if (deviceId === ARM2_ID) {
        callback({
          latitude: 45.4661989 + delta,
          longitude: -122.5782375 + delta,
        });
      } else if (deviceId === ARM3_ID) {
        callback({
          latitude: 45.4661989 + delta,
          longitude: -122.5782375 + delta + delta,
        });
      }
    }
    return () => {};
  }
}
