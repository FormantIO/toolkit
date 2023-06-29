import { sendAppMessage } from "./sendAppMessage";

export function goToTime(date: Date): void {
  sendAppMessage({
    type: "go_to_time",
    time: date.getTime(),
  });
}
