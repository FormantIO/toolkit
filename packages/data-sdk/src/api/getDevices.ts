import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { defined } from "../../../common/defined";

export async function getDevices(): Promise<Device[]> {
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
  return devices.items.map(
    (device: { id: string; name: string; tags?: Record<string, string> }) =>
      new Device(
        device.id,
        device.name,
        defined(Authentication.currentOrganization) as string,
        device.tags
      )
  );
}
