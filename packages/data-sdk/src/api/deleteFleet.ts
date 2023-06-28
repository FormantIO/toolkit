import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function deleteFleet(id: string): Promise<void> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}`, {
    method: "DELETE",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
}
