import { sendAppMessage } from "./sendAppMessage";

export function disableAnalyticsBottomBar() {
  sendAppMessage({
    type: "hide_analytics_date_picker",
  });
}
