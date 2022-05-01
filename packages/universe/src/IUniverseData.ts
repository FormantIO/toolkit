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
  configuration: {
    type: string;
    mapTopic?: string;
    validation?: {
      schemaId: string;
    };
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
  getDeviceContexts(): { deviceName: string; deviceId: string }[];
  getDeviceContextName(deviceId: string): string | undefined;
  getTelemetryStreamType(
    deviceId: string,
    streamName: string
  ): string | undefined;
  getTelemetryStreams(deviceId: string): ITelemetryStream[];
  getTeleopRosStreams(deviceId: string): ITelemetryRosStream[];
  getUrdfs(deviceId: string): Promise<string[]>;
  getHardwareStreams(deviceId: string): IHardwareStream[];
  subscribeToPointCloud(
    source: UniverseDataSource,
    callback: (data: IRtcPointCloud) => void
  ): () => void;
  subscribeToGeometry(
    source: UniverseDataSource,
    callback: (data: IMarker3DArray) => void
  ): () => void;
  subscribeToJointState(
    source: UniverseDataSource,
    callback: (data: IJointState) => void
  ): () => void;
  subscribeToMap(
    source: UniverseDataSource,
    callback: (data: IMap) => void
  ): () => void;
  subscribeToVideo(
    source: UniverseDataSource,
    callback: (data: IH264VideoFrame) => void
  ): () => void;
  subscribeToTransformTree(
    source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void;
  subscribeToLocation(
    source: UniverseDataSource,
    callback: (data: ILocation) => void
  ): () => void;
}
