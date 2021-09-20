import { Authentication, Fleet } from "@formant/data-sdk";

import "../../../css/formant.css";
import "./style.css";
import { definedAndNotNull } from "../../../packages/common/defined";
import "../../../packages/ui-sdk-joystick/src/main";
import "../../../packages/ui-sdk-realtime-player/src/main";
import { Authentication, Fleet } from "@formant/data-sdk";

const elBtnConnect = definedAndNotNull(
  document.querySelector("button")
) as HTMLElement;
const elLog = definedAndNotNull(document.querySelector("#log")) as HTMLElement;
const elPlayer = definedAndNotNull(
  document.querySelector("formant-realtime-player")
) as HTMLElement;
const elJoystick = definedAndNotNull(
  document.querySelector("formant-joystick")
) as HTMLElement;
const elSection = definedAndNotNull(
  document.querySelector("section")
) as HTMLElement;

function log(msg: string) {
  elLog.innerHTML = msg + "<br>" + elLog.innerHTML;
}

elBtnConnect.addEventListener("click", async () => {
  // hide intro
  elSection.style.display = "none";
  elLog.style.display = "block";

  // start connecting
  log("Waiting for authentication ...");
  if (!(await Authentication.waitTilAuthenticated())) {
    log("Not Authenticated");
    return;
  }

  const device = await Fleet.getCurrentDevice();
  debugger;
  log("Currently looking at <b>" + device.name + "</b>");
  log("Getting a realtime connection ... ");
  await device.startRealtimeConnection();
  log("Connected");

  // show the player
  elPlayer.style.display = "block";

  // show joysticks and connec them up
  elJoystick.style.display = "block";
  elJoystick.addEventListener("joystick", (e) => {
    log(JSON.stringify(e.detail));
  });
});
