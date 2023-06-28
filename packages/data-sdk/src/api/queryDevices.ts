import { IDeviceQuery } from "../model/IDeviceQuery";
import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function queryDevices(query: IDeviceQuery): Promise<Device[]> {
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

  return devices.items.map(
    (_: any) => new Device(_.id, _.name, _.organizationId, _.tags)
  );
}
