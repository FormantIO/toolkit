import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IAnalyticsModule } from "../model/IAnalyticsModule";

export async function getAnalyticsModules() {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const response = await fetch(
    `${FORMANT_API_URL}/v1/admin/analytics-modules`,
    {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  return (await response.json()).items as IAnalyticsModule;
}
