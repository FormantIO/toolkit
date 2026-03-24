import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

export async function getModuleConfiguration(
  id: string
): Promise<string | undefined> {
  const response = await fetch(`${DataSdk.adminApi}/module-configurations/${id}`, {
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
  });
  const moduleConfiguration = await response.json();
  return moduleConfiguration.configuration;
}
