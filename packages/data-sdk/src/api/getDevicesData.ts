import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { IDevice } from "../model/IDevice";

export async function getDevicesData(): Promise<IDevice[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/device-details/query`, {
    method: "POST",
    body: JSON.stringify({ enabled: true, type: "default" }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const devices = await data.json();
  return devices.items;
}
