import { IDeviceQuery } from "../model/IDeviceQuery";
import { Device } from "../devices/Device";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { ITags } from "../model/ITags";

interface IDeviceListItem {
  id: string;
  name: string;
  organizationId: string;
  tags: ITags;
}

export async function queryDevices(query: IDeviceQuery): Promise<Device[]> {
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

  return (devices.items as IDeviceListItem[]).map(
    (device) =>
      new Device(device.id, device.name, device.organizationId, device.tags)
  );
}
