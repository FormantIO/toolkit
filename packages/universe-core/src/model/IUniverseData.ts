import { IJointState } from "../../../data-sdk/src/model/IJointState";
import { ILocation } from "../../../data-sdk/src/model/ILocation";
import { IMarker3DArray } from "../../../data-sdk/src/model/IMarker3DArray";
import { ITransformNode } from "../../../data-sdk/src/model/ITransformNode";
import { INumericSetEntry } from "../../../data-sdk/src/model/INumericSetEntry";
import { IBitset } from "../../../data-sdk/src/model/IBitset";
import { IOdometry } from "./IOdometry";
import { IPose } from "./IPose";
import { IPcd } from "./IPcd";
import { IGridMap } from "./IGridMap";

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

export interface Interaction {
  deviceId: string;
  id: string;
  name: string;
  description: string;
  icon: string;
}

export interface RealtimeButtonConfiguration {
  streamName: string;
}
export interface IUniverseData {
  addInteraction: (interaction: Interaction) => void;

  removeInteraction: (id: string) => void;

  getInteractions: () => Interaction[];

  addInteractionsChangedListener: (
    callback: (interactions: Interaction[]) => void
  ) => () => void;

  addInteractionListener: (
    callback: (interaction: Interaction) => void
  ) => () => void;

  setTime(time: number | "live"): void;

  getLatestTransformTrees(deviceId: string): Promise<
    {
      streamName: string;
      transformTree: ITransformNode;
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
    callback: (data: IPcd) => void
  ): CloseSubscription;

  subscribeToOdometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IOdometry) => void
  ): CloseSubscription;

  subscribeToPose(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IPose) => void
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

  subscribeToGridMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IGridMap) => void
  ): CloseSubscription;

  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (frame: HTMLCanvasElement) => void
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

  subscribeToJson<T>(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: T) => void
  ): CloseSubscription;

  subscribeToText(
    deviceId: string,
    source: UniverseDataSource,
    callback: (text: string) => void
  ): CloseSubscription;

  subscribeToNumeric(
    deviceId: string,
    source: UniverseDataSource,
    callback: (num: [number, number][]) => void
  ): CloseSubscription;

  subscribeToNumericSet(
    deviceId: string,
    source: UniverseDataSource,
    callback: (entry: [number, INumericSetEntry[]][]) => void
  ): CloseSubscription;

  getStatistics(): Promise<IUniverseStatistics>;

  subscribeDataSourceStateChange(
    deviceId: string,
    source: UniverseDataSource,
    onDataSourceStateChange?: (state: DataSourceState) => void
  ): CloseSubscription;

  subscribeToImage(
    deviceId: string,
    source: UniverseDataSource,
    callback: (image: HTMLImageElement | HTMLCanvasElement) => void
  ): CloseSubscription;

  subcribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (frame: HTMLVideoElement) => void
  ): CloseSubscription;

  sendRealtimePose(
    deviceId: string,
    streamName: string,
    pose: IPose
  ): Promise<void>;

  sendRealtimeBoolean(
    deviceId: string,
    streamName: string,
    value: boolean
  ): Promise<void>;

  sendRealtimeBitset(
    deviceId: string,
    streamName: string,
    bitset: IBitset
  ): Promise<void>;

  sendCommand(deviceId: string, name: string, data?: string): Promise<void>;
}
