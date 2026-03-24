import { IFleet } from "../model/IFleet";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function listFleets(): Promise<IFleet[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/fleets`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const fleets = await data.json();
  return fleets.items;
}
