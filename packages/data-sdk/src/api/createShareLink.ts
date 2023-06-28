import { IShare } from "../model/IShare";
import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { serializeHash } from "../utils/serializeHash";

import { getViews } from "./getViews";

/**
 * @param scope is required
 * @param time is required
 * @param view View name
 * @returns
 * Share link
 * @example
 * // Body
 * const link = await Fleet.createShareLink({
 *     delegateTeleop: false
 *     message: "See bot in action",
 *     scope: {
 *       deviceIds: ["d64520a6-a308-4a59-9267-b7f8a7bfc7ab"],
 *       start: "2023-04-04T19:51:47.125Z",
 *       end: "2023-04-04T20:51:47.125Z"
 *      },
 *      time: "2023-04-04T20:21:47.125Z",
 *      userName: "User",
 *   });
 */
export async function createShareLink(share: IShare, view: string) {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const views = await getViews();

  const selectedView = views.filter((_) => _.name === view);

  if (selectedView.length === 0) {
    console.warn("View does not exist or it is misspell");
    return null;
  }

  const response = await fetch(`${FORMANT_API_URL}/v1/admin/shares`, {
    method: "POST",
    body: JSON.stringify(share),
    headers: {
      "Content-Type": "application/json",
      Authorization: "Bearer " + Authentication.token,
    },
  });
  const origin = FORMANT_API_URL.replace("api", "app");
  const { code } = await response.json();

  return `${origin}/shares/${code}#${serializeHash({
    viewId: selectedView[0].id,
  })}`;
}
