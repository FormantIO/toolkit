import { ISqlQuery } from "../model/ISqlQuery";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";

/**
 * @param taskColumns is required
 * @returns
 * All task reports
 * @example
 * // Body
 * const tasks = await getTaskReports({
 *     taskColumns: [
 *       {
 *         columns: [
 *           {
 *             dataType: "string",
 *             isNullable: true,
 *             name: "TYPE",
 *             tableName: "custom",
 *           },
 *         ],
 *         name: "DURATION_SECONDS",
 *         tableName: "TASK_REPORTS_CLEANING_MODE",
 *         yAxis: "DURATION_SECONDS",
 *       },
 *     ],
 *   });
 */

export async function getTaskReportRows(query: ISqlQuery) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }
  const response = await fetch(
    `${FORMANT_API_URL}/v1/queries/analytics/task-report-rows`,
    {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  return await response.json();
}
