import {
  IHardwareStream,
  ITelemetryRosStream,
  ITelemetryStream,
  IUniverseData,
  UniverseDataSource,
} from "@formant/universe";
import { IH264VideoFrame } from "@formant/universe/dist/types/data-sdk/src/model/IH264VideoFrame";
import { IJointState } from "@formant/universe/dist/types/data-sdk/src/model/IJointState";
import { ILocation } from "@formant/universe/dist/types/data-sdk/src/model/ILocation";
import { IMap } from "@formant/universe/dist/types/data-sdk/src/model/IMap";
import { IMarker3DArray } from "@formant/universe/dist/types/data-sdk/src/model/IMarker3DArray";
import { IRtcPointCloud } from "@formant/universe/dist/types/data-sdk/src/model/IRtcPointCloud";
import { ITransformNode } from "@formant/universe/dist/types/model/ITransformNode";

export const SPOT_ID = "abc";
export const ARM1_ID = "asdfadsfas";
export const ARM2_ID = "124fasd";
export const ARM3_ID = "77hrtesgdafdsh";

export class SimulatedUniverseData implements IUniverseData {
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
    _callback: (data: IRtcPointCloud) => void
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
    _callback: (data: IJointState) => void
  ): () => void {
    return () => {};
  }

  subscribeToMap(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IMap) => void
  ): () => void {
    return () => {};
  }

  subscribeToVideo(
    _deviceId: string,
    _source: UniverseDataSource,
    _callback: (data: IH264VideoFrame) => void
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
          callback({
            latitude: 45.4661989,
            longitude: -122.5782375 + 2 * delta * Math.sin(Date.now() / 1000),
          });
        }, 1000);
      } else if (deviceId === ARM1_ID) {
        window.setInterval(() => {
          callback({
            latitude: 45.4661989 + delta,
            longitude: -122.5782375,
          });
        }, 1000);
      } else if (deviceId === ARM2_ID) {
        window.setInterval(() => {
          callback({
            latitude: 45.4661989 + delta,
            longitude: -122.5782375 + delta,
          });
        }, 1000);
      } else if (deviceId === ARM3_ID) {
        window.setInterval(() => {
          callback({
            latitude: 45.4661989 + delta,
            longitude: -122.5782375 + delta + delta,
          });
        }, 1000);
      }
    }
    return () => {};
  }
}
