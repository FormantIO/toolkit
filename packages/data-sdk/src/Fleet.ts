import { Authentication } from "./Authentication";
import { getDevice } from "./api/getDevice";
import { getDevices } from "./api/getDevices";
import { getLatestTelemetry } from "./api/getLatestTelemetry";
import { getStreams } from "./api/getStreams";
import { queryDevices } from "./api/queryDevices";
import { queryTelemetry } from "./api/queryTelemetry";
import { IDevice } from "./message-bus/listeners/EmbeddedAppMessage";
import { Device } from "./devices/Device";
import { PeerDevice } from "./devices/PeerDevice";

/**
 * Device context and query helpers for embedded custom modules.
 * Not related to the admin-api /fleets resource (removed).
 */
export class Fleet {
  static defaultDeviceId: string | undefined;
  static knownContext: WeakRef<Device>[] = [];
  static groupDevices: IDevice[] | undefined;

  static async setDefaultDevice(deviceId: string) {
    Fleet.defaultDeviceId = deviceId;
  }

  static async getCurrentDevice(): Promise<Device> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    if (!Fleet.defaultDeviceId) {
      throw new Error("No known default device");
    }

    return Fleet.getDevice(Fleet.defaultDeviceId);
  }

  static async getPeerDevice(url: string): Promise<PeerDevice> {
    const peer = new PeerDevice(url);
    peer.id = await peer.getDeviceId();
    return peer;
  }

  static async getDevice(deviceId: string): Promise<Device> {
    const context = await getDevice(deviceId);
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static setGroupDevices(devices: IDevice[]): void {
    Fleet.groupDevices = devices;
    if (devices.length > 0 && !Fleet.defaultDeviceId) {
      Fleet.setDefaultDevice(devices[0].id);
    }
  }

  static getGroupDevices(): IDevice[] | undefined {
    return Fleet.groupDevices;
  }

  static async getGroupDevicesAsDeviceInstances(): Promise<Device[]> {
    if (!Fleet.groupDevices || Fleet.groupDevices.length === 0) {
      return [];
    }
    return Promise.all(Fleet.groupDevices.map((d) => Fleet.getDevice(d.id)));
  }

  static getDevices = getDevices;
  static queryDevices = queryDevices;
  static queryTelemetry = queryTelemetry;
  static getLatestTelemetry = getLatestTelemetry;
  static getStreams = getStreams;
}
