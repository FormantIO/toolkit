import { IView } from "../model/IView";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getViews(): Promise<IView[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${DataSdk.adminApi}/views`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const views = await response.json();
  return views.items;
}
