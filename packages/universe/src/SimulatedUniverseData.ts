import { IH264VideoFrame } from "../../data-sdk/src/model/IH264VideoFrame";
import { IJointState } from "../../data-sdk/src/model/IJointState";
import { ILocation } from "../../data-sdk/src/model/ILocation";
import { IMap } from "../../data-sdk/src/model/IMap";
import { IMarker3DArray } from "../../data-sdk/src/model/IMarker3DArray";
import { IStreamCurrentValue } from "../../data-sdk/src/model/IStreamCurrentValue";
import { ITransformNode } from "../../data-sdk/src/model/ITransformNode";
import {
  IHardwareStream,
  ITelemetryRosStream,
  ITelemetryStream,
  IUniverseData,
  UniverseDataSource,
} from "./IUniverseData";
import { IRtcPointCloud } from "./layers/IRtcPointCloud";

export class SimulatedUniverseData implements IUniverseData {
  async getTransformTrees(): Promise<{ name: string; transformTree: any }[]> {
    return [];
  }

  async getLocations(): Promise<IStreamCurrentValue<"location">[]> {
    return [];
  }

  getDeviceContexts(): { deviceName: string; deviceId: string }[] {
    return [{ deviceName: "My Robot", deviceId: "abc" }];
  }

  getDeviceContextName(_deviceId: string): string | undefined {
    return "My Robot";
  }

  getTelemetryStreamType(
    _deviceId: string,
    _streamName: string
  ): string | undefined {
    return undefined;
  }

  subscribeToDataSource<
    T extends
      | IRtcPointCloud
      | IMarker3DArray
      | IJointState
      | IMap
      | IH264VideoFrame
      | ITransformNode
  >(_source: UniverseDataSource, _callback: (data: T) => void): void {}

  get deviceId(): string {
    return "abc";
  }

  getTelemetryStreams(): ITelemetryStream[] {
    return [];
  }

  getTeleopRosStreams(): ITelemetryRosStream[] {
    return [];
  }

  async getUrdfs(_deviceId: string): Promise<string[]> {
    return [];
  }

  getHardwareStreams(): IHardwareStream[] {
    return [];
  }

  subscribeToTransformTree(
    _streamName: string,
    _callback: (data: ITransformNode) => void
  ): () => void {
    return () => {};
  }

  subscribeToLocation(
    _streamName: string,
    _callback: (data: ILocation) => void
  ): () => void {
    return () => {};
  }
}
