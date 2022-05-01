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

export interface IUniverseData {
  getTransformTrees(): Promise<
    {
      name: string;
      transformTree: any;
    }[]
  >;
  getLocations(): Promise<any>;
  getDeviceContexts(): { deviceName: string; deviceId: string }[];
  getDeviceContextName(deviceId: string): string | undefined;
  getTelemetryStreamType(
    deviceId: string,
    streamName: string
  ): string | undefined;
  subscribeToDataSource(
    source: UniverseDataSource,
    callback: (data: any) => void
  ): void;
  subscribeToTransformTree(
    streamName: string,
    callback: (data: any) => void
  ): () => void;
  subscribeToLocation(
    streamName: string,
    callback: (data: any) => void
  ): () => void;
  get deviceId(): string;
  getTelemetryStreams(): any[];
  getTeleopRosStreams(): any[];
  getUrdfs(deviceId: string): Promise<any>;
}
