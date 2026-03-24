import { IDevice } from "../model/IDevice";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function patchDevice(
  id: string,
  device: Partial<IDevice>
): Promise<IDevice> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices/${id}`, {
    method: "PATCH",
    body: JSON.stringify(device),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
