import { IQuery } from "../model/IQuery";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IStreamAggregateData } from "../model/IStreamAggregateData";

export async function aggregateTelemetry(query: IQuery) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const data = await fetch(`${FORMANT_API_URL}/v1/queries/queries`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });

  return (await data.json()).aggregates as IStreamAggregateData[];
}
