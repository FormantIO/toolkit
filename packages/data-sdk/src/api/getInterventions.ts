import { IEvent } from "../model/IEvent";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getInterventions(): Promise<IEvent[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const interventions = await fetch(`${DataSdk.adminApi}/intervention-requests`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
  });
  return (await interventions.json()).items as IEvent[];
}
