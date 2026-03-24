import { IDeviceQuery } from "../model/IDeviceQuery";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { IDevice } from "../model/IDevice";

export async function queryDevicesData(
  query: IDeviceQuery
): Promise<IDevice[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices/query`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const devices = await data.json();

  return devices.items;
}
