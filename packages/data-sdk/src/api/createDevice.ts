import { IDevice } from "../model/IDevice";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function createDevice(device: IDevice): Promise<IDevice> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices`, {
    method: "POST",
    body: JSON.stringify(device),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
