import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { IStreamColumn } from "../model/IStreamColumn";

/**
 * retrieves a list of all available data streams that can be used for running analytics.
 * This function takes no arguments and returns a list of stream names that can be used for analyzing data.
 * @example
 * // Returns
 *  [
 *    {
 *      streamName:  "$.agent.health",
          streamType :  "health"
        },
 {
 *      streamName:  "up.hours",
          streamType :  "numeric"
        }
 ]
 */
export async function getAnalyticStreams() {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const response = await fetch(
    `${FORMANT_API_URL}/v1/queries/analytics/streams`,
    {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    }
  );
  return (await response.json()).items as IStreamColumn[];
}
