import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IDevice } from "../model/IDevice";

export async function patchDevice(
  id: string,
  device: Partial<IDevice>
): Promise<IDevice> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/devices/${id}`, {
    method: "PATCH",
    body: JSON.stringify(device),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
