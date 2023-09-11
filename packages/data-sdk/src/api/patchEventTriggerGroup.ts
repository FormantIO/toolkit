import { IEventTriggerGroup } from "../model/IEventTriggerGroup";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function patchEventTriggerGroup(
  id: string,
  eventTrigger: Partial<IEventTriggerGroup>
): Promise<IEventTriggerGroup> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(
    `${FORMANT_API_URL}/v1/admin/event-trigger-groups/${id}`,
    {
      method: "PATCH",
      body: JSON.stringify(eventTrigger),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );

  return (await data.json()) as IEventTriggerGroup;
}
