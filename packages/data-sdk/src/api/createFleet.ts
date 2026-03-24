import { IFleet } from "../model/IFleet";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function createFleet(fleet: IFleet): Promise<IFleet> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/fleets`, {
    method: "POST",
    body: JSON.stringify(fleet),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
