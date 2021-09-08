import { Device } from "./Device";

export interface User {
  firstName: string;
  lastName: string;
  email: string;
  organizationId: string;
  id: string;
}

export class DataManager {
  static token: string | undefined;
  static currentUser: User | undefined;
  static defaultDeviceId: string | undefined;
  static waitingForAuth: ((result: boolean) => void)[] = [];
  static knownContext: WeakRef<Device>[] = [];

  static async login(_user: string, _password: string) {}

  static async setDefaultDevice(deviceId: string) {
    DataManager.defaultDeviceId = deviceId;
  }

  static async loginWithToken(token: string) {
    const tokenData = JSON.parse(atob(token.split(".")[1]));
    try {
      const data = await fetch(
        `https://api.formant.io/v1/admin/users/${tokenData.sub}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
            Authorization: "Bearer " + token,
          },
        }
      );
      DataManager.currentUser = await data.json();
      DataManager.token = token;
      DataManager.waitingForAuth.forEach((_) => _(true));
    } catch (e: any) {
      console.error(e);
      DataManager.waitingForAuth.forEach((_) => _(false));
    }
    DataManager.waitingForAuth = [];
  }

  static isAuthenticated(): boolean {
    return DataManager.token !== undefined;
  }

  static getCurrentUser(): User | undefined {
    return DataManager.currentUser;
  }

  static async getCurrentDevice(): Promise<Device> {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    if (!DataManager.defaultDeviceId) {
      throw new Error("No known default device");
    }

    const data = await fetch(
      `https://api.formant.io/v1/admin/devices/${DataManager.defaultDeviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + DataManager.token,
        },
      }
    );
    debugger;
    const device = await data.json();
    const name = device.name as string;
    const context = new Device(
      DataManager.token,
      DataManager.defaultDeviceId,
      name
    );
    DataManager.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevice(deviceId: string): Promise<Device> {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `https://api.formant.io/v1/admin/devices/${DataManager.defaultDeviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + DataManager.token,
        },
      }
    );
    debugger;
    const device = await data.json();
    const name = device.name as string;
    const context = new Device(DataManager.token, deviceId, name);
    DataManager.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevices(): Promise<Device[]> {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `https://api.formant.io/v1/admin/device-details/query`,
      {
        method: "POST",
        body: JSON.stringify({ enabled: true, type: "default" }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + DataManager.token,
        },
      }
    );
    const devices = await data.json();
    devices.items;
    return devices.items.map(
      (_: any) =>
        new Device(
          DataManager.token as string,
          _.id as string,
          _.name as string
        )
    );
  }

  static async waitTilAuthenticated(): Promise<boolean> {
    if (DataManager.token !== undefined) {
      return true;
    } else {
      return new Promise((resolve) => {
        DataManager.waitingForAuth.push(function (result: boolean) {
          resolve(result);
        });
      });
    }
  }
}

const urlParams = new URLSearchParams(window.location.search);

const urlDevice = urlParams.get("device");
if (urlDevice) {
  DataManager.setDefaultDevice(urlDevice);
}

const urlAuth = urlParams.get("auth");
if (urlAuth) {
  DataManager.loginWithToken(urlAuth);
}
