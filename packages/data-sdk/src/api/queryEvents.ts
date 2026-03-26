import { IEventQuery } from "../model/IEventQuery";
import { IEvent } from "../model/IEvent";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function queryEvents(query: IEventQuery): Promise<IEvent[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const data = await fetch(`${DataSdk.adminApi}/events/query`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });

  return (await data.json()).items as IEvent[];
}
