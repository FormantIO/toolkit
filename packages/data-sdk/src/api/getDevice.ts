import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function getDevice(deviceId: string): Promise<Device> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/devices/${deviceId}`, {
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
