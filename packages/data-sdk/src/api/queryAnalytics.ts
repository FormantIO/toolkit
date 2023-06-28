import { ISqlQuery } from "../model/ISqlQuery";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { ISqlResult } from "../model/ISqlResult";

/**
 *Retrieves all stream rows
 * @example
 * // Body
 * const analytics = await Fleet.queryAnalytics({
 *     aggregateLevel: "day",
 *     orderByColumn: "TIMESTAMP",
 *     streamColumns: [
 *       {
 *         streamName: "consumables_residual_percentage",
 *         streamType: "numeric set",
 *       },
 *     ],
 *   });
 * //Returns
 * {
 *    aggregates: [],
 *    columns: [
 *              {
 *                name: 'TIMESTAMP',
 *                isNullable: true,
 *                dataType: 'string',
 *                tableName: 'NUMERIC_SET_MAIN'
 *               }
 * ],
 *    items: [
 *              {
 *                axisLabel: "suction_blade",
 *                name: "consumables_residual_percentage",
 *                tableName: "NUMERIC_SET_TEST",
 *                time: "2020-04-20T08:00:00.000Z",
 *                type: "numeric set",
 *                unitLabel: "percent"
 *                }
 * ],
 *    rowCount: 14,
 *    rows: []
 *    sqlText: "SELECT dateadd(day, dayofweek(TIMESTAMP), to_timestamp_tz('4/20/2020')) AS TIMESTAMP, SUM(VALUE)"
 * }
 */
export async function queryAnalytics(query: ISqlQuery) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(`${FORMANT_API_URL}/v1/queries/analytics`, {
    method: "POST",
    body: JSON.stringify(query),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  return (await response.json()) as ISqlResult;
}
