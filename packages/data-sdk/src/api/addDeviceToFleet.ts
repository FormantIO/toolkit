import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function addDeviceToFleet(deviceId: string, fleetId: string) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/devices/${deviceId}`, {
    method: "PATCH",
    body: JSON.stringify({ fleetId }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
