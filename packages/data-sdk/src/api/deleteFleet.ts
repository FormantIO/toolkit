import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function deleteFleet(id: string): Promise<void> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  await fetch(`${DataSdk.adminApi}/fleets/${id}`, {
    method: "DELETE",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
}
