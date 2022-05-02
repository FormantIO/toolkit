import { IH264VideoFrame } from "../../data-sdk/src/model/IH264VideoFrame";
import { IJointState } from "../../data-sdk/src/model/IJointState";
import { ILocation } from "../../data-sdk/src/model/ILocation";
import { IMap } from "../../data-sdk/src/model/IMap";
import { IMarker3DArray } from "../../data-sdk/src/model/IMarker3DArray";
import { ITransformNode } from "../../data-sdk/src/model/ITransformNode";
import { IRtcPointCloud } from "../../data-sdk/src/model/IRtcPointCloud";

export interface UniverseRosDataSource {
  id: string;
  sourceType: "realtime";
  rosTopicName: string;
  rosTopicType: string;
}

export interface UniverseHardwareDataSource {
  id: string;
  sourceType: "hardware";
  rtcStreamName: string;
}

export interface UniverseTelemetrySource {
  id: string;
  sourceType: "telemetry";
  streamName: string;
  streamType: string;
}

export type UniverseDataSource =
  | UniverseRosDataSource
  | UniverseTelemetrySource
  | UniverseHardwareDataSource;

export interface ITelemetryStream {
  name: string;
  disabled?: boolean;
  validation?: {
    schemaId: string;
  };
  configuration: {
    type: string;
    mapTopic?: string;
  };
}
export interface ITelemetryRosStream {
  topicName: string;
  topicType: string;
}
export interface IHardwareStream {
  rtcStreamType: string;
  name: string;
}

export interface IUniverseData {
  getLatestTransformTrees(deviceId: string): Promise<
    {
      streamName: string;
      transformTree: any;
    }[]
  >;
  getLatestLocations(
    deviceId: string
  ): Promise<{ streamName: string; location: ILocation }[]>;
  getDeviceContexts(): Promise<{ deviceName: string; deviceId: string }[]>;
  getDeviceContextName(deviceId: string): Promise<string | undefined>;
  getTelemetryStreamType(
    deviceId: string,
    streamName: string
  ): Promise<string | undefined>;
  getTelemetryStreams(deviceId: string): Promise<ITelemetryStream[]>;
  getTeleopRosStreams(deviceId: string): Promise<ITelemetryRosStream[]>;
  getUrdfs(deviceId: string): Promise<string[]>;
  getHardwareStreams(deviceId: string): Promise<IHardwareStream[]>;
  subscribeToPointCloud(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IRtcPointCloud) => void
  ): () => void;
  subscribeToGeometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IMarker3DArray) => void
  ): () => void;
  subscribeToJointState(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IJointState) => void
  ): () => void;
  subscribeToMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IMap) => void
  ): () => void;
  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IH264VideoFrame) => void
  ): () => void;
  subscribeToTransformTree(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void;
  subscribeToLocation(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ILocation) => void
  ): () => void;
}
