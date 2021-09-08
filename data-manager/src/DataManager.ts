import { DeviceContext } from "./DeviceContext";

export interface User {
  firstName: string;
  lastName: string;
  email: string;
  organizationId: string;
  id: string;
}

export interface Device {
  /**
   * A human readable name of a device
   */
  name: string;
  /**
   *  A unique ID representing a device
   */
  id: string;
}

export class DataManager {
  static token: string | undefined;
  static currentUser: User | undefined;
  static defaultDeviceId: string | undefined;
  static waitingForAuth: ((result: boolean) => void)[] = [];
  static knownContext: WeakRef<DeviceContext>[] = [];

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

  static getCurrentDeviceContext(): DeviceContext {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    if (!DataManager.defaultDeviceId) {
      throw new Error("No known default device");
    }
    const context = new DeviceContext(
      DataManager.token,
      DataManager.defaultDeviceId
    );
    DataManager.knownContext.push(new WeakRef(context));
    return context;
  }

  static getDeviceContext(deviceId: string): DeviceContext {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    const context = new DeviceContext(DataManager.token, deviceId);
    DataManager.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevices(): Promise<Device[]> {
    if (!DataManager.token) {
      throw new Error("Not authenticated");
    }
    return [];
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
