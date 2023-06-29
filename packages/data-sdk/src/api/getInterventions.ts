import { IEvent } from "../model/IEvent";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function getInterventions(): Promise<IEvent[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const interventions = await fetch(
    `${FORMANT_API_URL}/v1/admin/intervention-requests`,
    {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  return (await interventions.json()).items as IEvent[];
}
