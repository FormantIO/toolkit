import { IFleet } from "../model/IFleet";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function listFleets(): Promise<IFleet[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const fleets = await data.json();
  return fleets.items;
}
