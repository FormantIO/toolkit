import { IView } from "../model/IView";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function patchView(view: IView): Promise<IView> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${DataSdk.adminApi}/views/${view.id}`, {
    method: "PATCH",
    body: JSON.stringify(view),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return (await response.json()) as IView;
}
