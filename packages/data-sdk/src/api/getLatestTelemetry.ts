import { IStreamCurrentValue } from "../model/IStreamCurrentValue";
import { IStreamTypeMap } from "../model/IStreamTypeMap";
import { DataSdk } from "../DataSdk";
import { Authentication } from "../Authentication";

export async function getLatestTelemetry(
  ...ids: (string | string[])[]
): Promise<IStreamCurrentValue<keyof IStreamTypeMap>[]> {
  const deviceIds = ids.flat().filter((_) => !!_);

  if (deviceIds.length === 0) {
    return [];
  }

  const data = await fetch(
    `${DataSdk.queryApi}/stream-current-value`,
    {
      method: "POST",
      body: JSON.stringify({
        deviceIds,
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  const telemetry = await data.json();
  return telemetry.items;
}
