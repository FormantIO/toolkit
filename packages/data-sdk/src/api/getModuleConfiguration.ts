import { FORMANT_API_URL } from "../config";
import { Authentication } from "../Authentication";

export async function getModuleConfiguration(
  id: string
): Promise<string | undefined> {
  const response = await fetch(
    `${FORMANT_API_URL}/v1/admin/module-configurations/${id}`,
    {
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  const moduleConfiguration = await response.json();
  return moduleConfiguration.configuration;
}
