import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IDevice } from "../model/IDevice";

export async function getDevicesData(): Promise<IDevice[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/device-details/query`, {
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
