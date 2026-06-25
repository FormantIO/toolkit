import { defined } from "../../common/defined";
import { Authentication } from "./Authentication";
import { DataSdk } from "./DataSdk";
import { getAnalyticsRows } from "./api/getAnalyticsRows";
import { getDevice } from "./api/getDevice";
import { getDevices } from "./api/getDevices";
import { getEvent } from "./api/getEvent";
import { getFileUrl } from "./api/getFileUrl";
import { getLatestTelemetry } from "./api/getLatestTelemetry";
import { getOnlineDevices } from "./api/getOnlineDevices";
import { getStreams } from "./api/getStreams";
import { getViews } from "./api/getViews";
import { patchView } from "./api/patchView";
import { queryDevices } from "./api/queryDevices";
import { queryEvents } from "./api/queryEvents";
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

    const data = await fetch(`${DataSdk.adminApi}/device-details/query`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });

    const devices = await data.json();
    const device = devices.items.find(
      (_: { id: string }) => _.id === Fleet.defaultDeviceId
    );
    const name = device.name as string;
    const context = new Device(
      Fleet.defaultDeviceId,
      name,
      defined(Authentication.currentOrganization) as string,
      device.tags
    );
    Fleet.knownContext.push(new WeakRef(context));
    return context;
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

  static getAnalyticsRows = getAnalyticsRows;
  static getDevices = getDevices;
  static getEvent = getEvent;
  static getFileUrl = getFileUrl;
  static getOnlineDevices = getOnlineDevices;
  static getViews = getViews;
  static patchView = patchView;
  static queryDevices = queryDevices;
  static queryEvents = queryEvents;
  static queryTelemetry = queryTelemetry;
  static getLatestTelemetry = getLatestTelemetry;
  static getStreams = getStreams;
}
