import { IJointState } from "../../model/IJointState";
import { ILocation } from "../../model/ILocation";
import { IMarker3DArray } from "../../model/IMarker3DArray";
import { ITransformNode } from "../../model/ITransformNode";
import { INumericSetEntry } from "../../model/INumericSetEntry";
import { IBitset } from "../../model/IBitset";
import { IUniverseOdometry } from "./IUniverseOdometry";
import { IPose } from "./IPose";
import { IUniverseGridMap } from "./IUniverseGridMap";
import { IUniversePointCloud } from "./IUniversePointCloud";
import { IUniversePath } from "./IUniversePath";
import { StreamType } from "../../model/StreamType";

export type DataSourceState =
  | "missing_data"
  | "connected"
  | "disconnected"
  | "connecting"
  | "disconnecting";

export const NoData = Symbol("no_data");
export const LoadingData = Symbol("loading_data");
export const FailedData = Symbol("failed_data");
export type DataStatus = Symbol;

export type RtcStreamType =
  | "ping"
  | "pong"
  | "stream-control"
  | "streams-info"
  | "agent-info"
  | "numeric"
  | "boolean"
  | "bitset"
  | "twist"
  | "compressed-image"
  | "h264-video-frame"
  | "audio-chunk"
  | "pose"
  | "goal-id"
  | "joint-state"
  | "pose-with-covariance"
  | "point-cloud"
  | "marker-array"
  | "point"
  | "json-string";

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
  streamType?: StreamType;
  latestDataPoint?: boolean;
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
export interface IRealtimeStream {
  rtcStreamType: RtcStreamType;
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

  setTime(time: Date | "live"): void;

  getTime(): Date | "live";

  getTimeMs(): number;

  getLatestTransformTrees(deviceId: string): Promise<
    {
      streamName: string;
      transformTree: ITransformNode;
    }[]
  >;

  clearWorkerPool(): void;

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

  getHardwareStreams(deviceId: string): Promise<IRealtimeStream[]>;

  subscribeToBitset(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IBitset | DataStatus) => void
  ): CloseSubscription;

  subscribeToPointCloud(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniversePointCloud | DataStatus) => void
  ): CloseSubscription;

  subscribeToOdometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniverseOdometry | DataStatus) => void,
    trail?: number
  ): CloseSubscription;

  subscribeToPath(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniversePath | DataStatus) => void
  ): CloseSubscription;

  subscribeToPose(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IPose | DataStatus) => void
  ): CloseSubscription;

  subscribeToGeometry(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IMarker3DArray | DataStatus) => void
  ): CloseSubscription;

  subscribeToJointState(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IJointState | DataStatus) => void
  ): CloseSubscription;

  subscribeToGridMap(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: IUniverseGridMap | DataStatus) => void
  ): CloseSubscription;

  subscribeToVideo(
    deviceId: string,
    source: UniverseDataSource,
    callback: (frame: HTMLCanvasElement | DataStatus) => void
  ): CloseSubscription;

  subscribeToTransformTree(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ITransformNode | DataStatus) => void
  ): CloseSubscription;

  subscribeToLocation(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: ILocation | DataStatus) => void
  ): CloseSubscription;

  subscribeToJson<T>(
    deviceId: string,
    source: UniverseDataSource,
    callback: (data: T | DataStatus) => void
  ): CloseSubscription;

  subscribeToText(
    deviceId: string,
    source: UniverseDataSource,
    callback: (text: string | DataStatus) => void
  ): CloseSubscription;

  subscribeToNumeric(
    deviceId: string,
    source: UniverseDataSource,
    callback: (num: [number, number][] | DataStatus) => void
  ): CloseSubscription;

  subscribeToNumericSet(
    deviceId: string,
    source: UniverseDataSource,
    callback: (entry: [number, INumericSetEntry[]][] | DataStatus) => void
  ): CloseSubscription;

  getStatistics(): Promise<IUniverseStatistics>;

  subscribeDataSourceStateChange(
    deviceId: string,
    source: UniverseDataSource,
    onDataSourceStateChange?: (state: DataSourceState | DataStatus) => void
  ): CloseSubscription;

  subscribeToImage(
    deviceId: string,
    source: UniverseDataSource,
    callback: (image: HTMLCanvasElement | DataStatus) => void
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
