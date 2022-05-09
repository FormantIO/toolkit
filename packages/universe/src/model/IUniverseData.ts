import { IH264VideoFrame } from "../../../data-sdk/src/model/IH264VideoFrame";
import { IJointState } from "../../../data-sdk/src/model/IJointState";
import { ILocation } from "../../../data-sdk/src/model/ILocation";
import { IMap } from "../../../data-sdk/src/model/IMap";
import { IMarker3DArray } from "../../../data-sdk/src/model/IMarker3DArray";
import { ITransformNode } from "../../../data-sdk/src/model/ITransformNode";
import { IRtcPointCloud } from "../../../data-sdk/src/model/IRtcPointCloud";

export type DataSourceState =
  | "missing_data"
  | "connected"
  | "disconnected"
  | "connecting"
  | "disconnecting";

export type CloseSubscription = () => void;

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

export interface IUniverseStatistics {
  rtcDevices: {
    deviceId: string;
    deviceName: string;
    totalRtcDataBytes: number;
  }[];
}

export interface IUniverseData {
  setTime(time: number): void;

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
  ): CloseSubscription;

  subscribeToGeometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IMarker3DArray) => void
  ): CloseSubscription;

  subscribeToJointState(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IJointState) => void
  ): CloseSubscription;

  subscribeToMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IMap) => void
  ): CloseSubscription;

  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (
      data:
        | { type: "frame"; frame: IH264VideoFrame }
        | { type: "url"; url: string }
    ) => void
  ): CloseSubscription;

  subscribeToTransformTree(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): CloseSubscription;

  subscribeToLocation(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ILocation) => void
  ): CloseSubscription;

  subscribeToJson(
    deviceId: string,
    source: UniverseDataSource,
    callback: (
      data:
        | { type: "url"; url: string }
        | { type: "json"; data: any }
        | { type: "raw_json"; data: string }
    ) => void
  ): CloseSubscription;

  subscribeToText(
    deviceId: string,
    source: UniverseDataSource,
    callback: (
      data: { type: "url"; url: string } | { type: "text"; data: string }
    ) => void
  ): CloseSubscription;

  getStatistics(): Promise<IUniverseStatistics>;

  subscribeDataSourceStateChange(
    deviceId: string,
    source: UniverseDataSource,
    onDataSourceStateChange?: (state: DataSourceState) => void
  ): CloseSubscription;
}
