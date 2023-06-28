import { IView } from "../model/IView";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

export async function getViews(): Promise<IView[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${FORMANT_API_URL}/v1/admin/views`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const views = await response.json();
  return views.items;
}
