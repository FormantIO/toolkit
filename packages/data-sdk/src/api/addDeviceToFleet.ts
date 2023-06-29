import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function addDeviceToFleet(deviceId: string, fleetId: string) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/devices/${deviceId}`, {
    method: "PATCH",
    body: JSON.stringify({ fleetId }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
