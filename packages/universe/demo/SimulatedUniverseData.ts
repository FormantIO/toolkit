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
import { IRtcPointCloud } from "../src/layers/IRtcPointCloud";

export class SimulatedUniverseData implements IUniverseData {
  async getLatestTransformTrees(): Promise<
    { streamName: string; transformTree: ITransformNode }[]
  > {
    return [
      {
        streamName: "spotTf",
        transformTree: await fetch(
          "https://upload.formant.io/0d29f656-cc1c-4b9e-baad-199cfa1fcced/b0990d5a-cdff-4c3c-ab71-c6c72be385ad/2022/05/1/af202124-bab3-4191-8948-458dc96efaef"
        ).then((_) => _.json() as ITransformNode),
      },
    ];
  }

  async getLatestLocations(): Promise<
    {
      streamName: string;
      location: ILocation;
    }[]
  > {
    return [];
  }

  getDeviceContexts(): { deviceName: string; deviceId: string }[] {
    return [
      { deviceName: "Spot-9000", deviceId: "abc" },
      { deviceName: "Roboarm 1", deviceId: "asdfcsad" },
      { deviceName: "Roboarm 2", deviceId: "dfhgf" },
      { deviceName: "Roboarm 3", deviceId: "vbmvb" },
    ];
  }

  getDeviceContextName(deviceId: string): string | undefined {
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

  getTelemetryStreamType(
    _deviceId: string,
    streamName: string
  ): string | undefined {
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

  get deviceId(): string {
    return "abc";
  }

  getTelemetryStreams(_deviceId: string): ITelemetryStream[] {
    return [
      {
        name: "spotTf",
        configuration: {
          type: "transform tree",
        },
      },
    ];
  }

  getTeleopRosStreams(_deviceId: string): ITelemetryRosStream[] {
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

  async getUrdfs(_deviceId: string): Promise<string[]> {
    return [
      "https://telemetry-prod-usw2-file-storage-service.s3.amazonaws.com/files/0d29f656-cc1c-4b9e-baad-199cfa1fcced/ff2b4e4f-80fa-4f30-b21a-b51b3538c4cc?AWSAccessKeyId=ASIA3QXDC4OAOOW32EXQ&Expires=1651461837&Signature=aOeBbkHKasFwPk%2Bp0K%2FYrB7bkqo%3D&x-amz-security-token=IQoJb3JpZ2luX2VjEFsaCXVzLXdlc3QtMiJGMEQCIF%2FE3FSpZ1wMA6zKrWs8PotRY%2BRPDdI3I%2FoZ0rzZVfYYAiBnEMX6K3nPimcBpBs2LIIzzAMlKd6wib3JmPGFFDae6yrSBAgkEAMaDDc5MTgyMzk2NzEwNCIMkpS4qnuAlN2QLMerKq8E0yvznfYrH1eRHQH2efHo53Yp8KtnjMWjfhcrclErok%2FWuencR4nnErBcUGJJ4QhCVoaDi4%2Bl0CPcCBHgczZSM6sJuATlcdn9mGsHWc6RYvJhYR%2FSffeq%2FM0zrR2tKeUkxiempcA3DpnzTOECbRLhPHKCdIanvpOPm1iq%2Bpa9vGiI0XVHd49L57fOCSpW8XmYLfer4Xnb1RMu%2FlFcnpHmD4zMK62Rk938K891xfdIi6JPUB0ee5MntaxI6Wrw2kOQ1J9MASicEDH%2F%2BCzipj6UDZ1R%2FlMS2Y405wN8%2BYICv0oAKuMFIfHaDRQR%2BPCnSsTuvCAGXEoJcW6122CoryvPHcUE4S6yDI9RmGR1SORnDJZdZyQfDAsuWtMrUqnaXvaL5ioegz1kZWlosbuYYPJqXDjri7KMppnGDYhKT6PBvWMBo%2BkPrk7VJpakU%2BU%2Bci4dQqr8g7HQ3jvYSZNJfw%2FhhHLKpHnxFgCJ5sENS3gY5rJM73X%2F91cpbE1sMN9KalL%2FQet4oRYZNTd3Tyv7iGVL14ejBWSkjKzegXwaBLzggaG9DWtHeZxHKUx0baLpcfcLoJyc%2F7BPWpOq7bfONBm%2BkPKAump2SNVwEyONZcUmt%2BjdaNsYZATAMk0ABnxW8aKiDutvPbmZ1fOVsdB8Nu2B2%2BX3wfs66UXqEzncLmsGQgowTwvUhQsjvyLsMokVMz%2BWgA0qJuhCGEwlZYgN%2B4zsE6tVGb5ooawimNwRo37i8DCI9reTBjqqAUMuymW4ktzSAlDxPLjAEIvKyQhkrxAngpVCaTiTx7g3p%2FH9Gj6frefydguCUQ%2FhGtlKnZY56IHksJNZGYOVqve9NJOKxdLGMkVdCoekKLNGKZ9c8oA95pn3BpB6kPswQjV%2B6CV3thymsAVmG2BDRo0iMhQDbpATD3v82%2FPzPj5wOdXJ0Y0NwRBcIa%2BWib%2FCGxuFcczdHUltTttNYwM3LL7stS2JySLVY62A",
    ];
  }

  getHardwareStreams(_deviceId: string): IHardwareStream[] {
    return [];
  }

  subscribeToTransformTree(
    _source: UniverseDataSource,
    callback: (data: ITransformNode) => void
  ): () => void {
    callback({
      url: "https://upload.formant.io/0d29f656-cc1c-4b9e-baad-199cfa1fcced/b0990d5a-cdff-4c3c-ab71-c6c72be385ad/2022/05/1/af202124-bab3-4191-8948-458dc96efaef",
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
