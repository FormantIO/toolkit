import { IDeviceQuery } from "../model/IDeviceQuery";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IDevice } from "../main";

export async function queryDevicesData(
  query: IDeviceQuery
): Promise<IDevice[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/devices/query`, {
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
