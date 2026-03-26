import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { IAnalyticsModule } from "../model/IAnalyticsModule";

export async function getAnalyticsModules() {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const response = await fetch(`${DataSdk.adminApi}/analytics-modules`, {
    method: "GET",
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return (await response.json()).items as IAnalyticsModule;
}
