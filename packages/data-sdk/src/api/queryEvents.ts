import { IEventQuery } from "../model/IEventQuery";
import { IEvent } from "../model/IEvent";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function queryEvents(query: IEventQuery): Promise<IEvent[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const data = await fetch(`${FORMANT_API_URL}/v1/admin/events/query`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });

  return (await data.json()).items as IEvent[];
}
