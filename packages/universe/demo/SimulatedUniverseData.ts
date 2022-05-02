import { IH264VideoFrame } from "../../data-sdk/src/model/IH264VideoFrame";
import { IJointState } from "../../data-sdk/src/model/IJointState";
import { ILocation } from "../../data-sdk/src/model/ILocation";
import { IMap } from "../../data-sdk/src/model/IMap";
import { IMarker3DArray } from "../../data-sdk/src/model/IMarker3DArray";
import { ITransformNode } from "../../data-sdk/src/model/ITransformNode";
import {
  IHardwareStream,
  ITelemetryRosStream,
  ITelemetryStream,
  IUniverseData,
  UniverseDataSource,
} from "../src/IUniverseData";
import { IRtcPointCloud } from "../../data-sdk/src/model/IRtcPointCloud";

export class SimulatedUniverseData implements IUniverseData {
  async getLatestTransformTrees(
    deviceId: string
  ): Promise<{ streamName: string; transformTree: ITransformNode }[]> {
    if (deviceId === "abc") {
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
    return [];
  }

  async getDeviceContexts(): Promise<
    { deviceName: string; deviceId: string }[]
  > {
    return [
      { deviceName: "Spot-9000", deviceId: "abc" },
      { deviceName: "Roboarm 1", deviceId: "asdfcsad" },
      { deviceName: "Roboarm 2", deviceId: "dfhgf" },
      { deviceName: "Roboarm 3", deviceId: "vbmvb" },
    ];
  }

  async getDeviceContextName(deviceId: string): Promise<string | undefined> {
    if (deviceId === "abc") {
      return "Spot-9000";
    }
    if (deviceId === "asdfcsad") {
      return "Roboarm 1";
    }
    if (deviceId === "dfhgf") {
      return "Roboarm 2";
    }
    if (deviceId === "vbmvb") {
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
    _source: UniverseDataSource,
    _callback: (data: IRtcPointCloud) => void
  ): () => void {
    return () => {};
  }

  subscribeToGeometry(
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
    _source: UniverseDataSource,
    _callback: (data: IJointState) => void
  ): () => void {
    return () => {};
  }

  subscribeToMap(
    _source: UniverseDataSource,
    _callback: (data: IMap) => void
  ): () => void {
    return () => {};
  }

  subscribeToVideo(
    _source: UniverseDataSource,
    _callback: (data: IH264VideoFrame) => void
  ): () => void {
    return () => {};
  }

  async getTelemetryStreams(_deviceId: string): Promise<ITelemetryStream[]> {
    return [
      {
        name: "spotTf",
        configuration: {
          type: "transform tree",
        },
      },
    ];
  }

  async getTeleopRosStreams(_deviceId: string): Promise<ITelemetryRosStream[]> {
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

  async getUrdfs(deviceId: string): Promise<string[]> {
    if (deviceId === "abc") {
      return ["https://formant-3d-models.s3.us-west-2.amazonaws.com/spot.zip"];
    }
    return ["https://formant-3d-models.s3.us-west-2.amazonaws.com/arm.zip"];
  }

  async getHardwareStreams(_deviceId: string): Promise<IHardwareStream[]> {
    return [];
  }

  subscribeToTransformTree(
    _source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void {
    callback({
      url: "https://formant-3d-models.s3.us-west-2.amazonaws.com/spotjoint_sit.json",
    });
    return () => {};
  }

  subscribeToLocation(
    _source: UniverseDataSource,
    _callback: (data: ILocation) => void
  ): () => void {
    return () => {};
  }
}
