import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getFleetDevices(id: string): Promise<void> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/fleets/${id}/devices`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const fleets = await data.json();
  return fleets.items;
}
