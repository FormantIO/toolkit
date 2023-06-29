import { FORMANT_API_URL } from "../config";
import { Authentication } from "../Authentication";

import { TelemetryResult } from "../model/TelemetryResult";

export async function getTelemetry(
  deviceIdOrDeviceIds: string | string[],
  streamNameOrStreamNames: string | string[],
  start: Date,
  end: Date,
  tags?: { [key in string]: string[] }
): Promise<TelemetryResult[]> {
  let deviceIds = deviceIdOrDeviceIds;
  if (!Array.isArray(deviceIdOrDeviceIds)) {
    deviceIds = [deviceIdOrDeviceIds];
  }
  let streamNames = streamNameOrStreamNames;
  if (!Array.isArray(streamNameOrStreamNames)) {
    streamNames = [streamNameOrStreamNames];
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/queries/queries`, {
    method: "POST",
    body: JSON.stringify({
      deviceIds,
      end: end.toISOString(),
      names: streamNames,
      start: start.toISOString(),
      tags,
    }),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const telemetry = await data.json();
  return telemetry.items;
}
