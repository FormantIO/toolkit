import { IEventTriggerGroup } from "../model/IEventTriggerGroup";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getAllEventTriggerGroup(): Promise<IEventTriggerGroup[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${DataSdk.adminApi}/event-trigger-groups`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });

  return (await data.json()).items as IEventTriggerGroup[];
}
