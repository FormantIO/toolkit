import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { IDevice } from "../model/IDevice";

export async function disableDevice(id: string): Promise<IDevice[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices/${id}/disable`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const response = await data.json();

  return response;
}
