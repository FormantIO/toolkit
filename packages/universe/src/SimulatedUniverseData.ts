import { IUniverseData, UniverseDataSource } from './IUniverseData';

export class SimulatedUniverseData implements IUniverseData {
  async getTransformTrees(): Promise<{ name: string; transformTree: any }[]> {
    return [];
  }

  async getLocations(): Promise<any> {

  }

  getDeviceContexts(): { deviceName: string; deviceId: string }[] {
    return [{ deviceName: 'My Robot', deviceId: 'abc' }];
  }

  getDeviceContextName(_deviceId: string): string | undefined {
    return 'My Robot';
  }

  getTelemetryStreamType(
    _deviceId: string,
    _streamName: string,
  ): string | undefined {

  }

  subscribeToDataSource(
    _source: UniverseDataSource,
    _callback: (data: any) => void,
  ): void {

  }

  subscribeToTransformTree(
    _streamName: string,
    _callback: (data: any) => void,
  ): () => void {
    return () => {};
  }

  subscribeToLocation(
    _streamName: string,
    _callback: (data: any) => void,
  ): () => void {
    return () => {};
  }

  get deviceId(): string {
    return 'abc';
  }
}
