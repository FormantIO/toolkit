import { IFleet } from "../model/IFleet";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getFleet(id: string): Promise<IFleet> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/fleets/${id}`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
