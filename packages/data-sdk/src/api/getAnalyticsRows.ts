import { ISqlQuery } from "../model/ISqlQuery";
import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";

/**
 * Retrieves all rows
 * sqlQuery is required
 * @param query
 * @returns
 */
export async function getAnalyticsRows(query: ISqlQuery) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${DataSdk.queryApi}/analytics/rows`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return await response.json();
}
