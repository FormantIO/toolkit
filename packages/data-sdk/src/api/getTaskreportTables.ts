import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { ITaskReportColumn } from "../model/ITaskReportColumn";

/**
 * retrieves a list of all available tables  that can be used to create task reports.
 * This function takes no arguments and returns a list of table names that can be used for creating task reports.
 * @returns List all available tables
 * @example
 * // Returns
 *[
 *    {
 *       name: "",
 *       tableName: "TASK_REPORTS_CLEANING_MODE",
 *       columns: [
 *                 {
 *                    name: "TYPE",
 *                    isNullable: true,
 *                    dataType: "string",
 *                    tableName: "custom"
 *                 }
 *                ]
 *    }
 *]
 */
export async function getTaskReportTables() {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const response = await fetch(
    `${FORMANT_API_URL}/v1/queries/analytics/task-reports`,
    {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  return (await response.json()).items as ITaskReportColumn[];
}
