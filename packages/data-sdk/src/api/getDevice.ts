import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getDevice(deviceId: string): Promise<Device> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices/${deviceId}`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const device = await data.json();
  const name = device.name as string;
  return new Device(deviceId, name, device.organizationId, device.tags);
}
