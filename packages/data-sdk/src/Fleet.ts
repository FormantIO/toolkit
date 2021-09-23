import { Device } from "./Device";
import { Authentication } from "./Authentication";
import { FORMANT_API_URL } from "./config";

export interface User {
  firstName: string;
  lastName: string;
  email: string;
  organizationId: string;
  id: string;
}

export class Fleet {
  static defaultDeviceId: string | undefined;
  static knownContext: WeakRef<Device>[] = [];

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

    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${Fleet.defaultDeviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const device = await data.json();
    const name = device.name as string;
    const context = new Device(
      Authentication.token,
      Fleet.defaultDeviceId,
      name,
      device.organizationId as string
    );
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevice(deviceId: string): Promise<Device> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${Fleet.defaultDeviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const device = await data.json();
    const name = device.name as string;
    const context = new Device(
      Authentication.token,
      deviceId,
      name,
      device.organizationId
    );
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevices(): Promise<Device[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/device-details/query`,
      {
        method: "POST",
        body: JSON.stringify({ enabled: true, type: "default" }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const devices = await data.json();
    devices.items;
    return devices.items.map(
      (_: any) =>
        new Device(
          Authentication.token as string,
          _.id as string,
          _.name as string,
          _.organizationId as string
        )
    );
  }
}
