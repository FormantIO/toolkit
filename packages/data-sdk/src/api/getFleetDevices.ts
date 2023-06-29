import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function getFleetDevices(id: string): Promise<void> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}/devices`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const fleets = await data.json();
  return fleets.items;
}
