import { IFleet } from "../model/IFleet";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function patchFleet(
  id: string,
  role: Partial<IFleet>
): Promise<IFleet> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}`, {
    method: "PATCH",
    body: JSON.stringify(role),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await data.json();
}
