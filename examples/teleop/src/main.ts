import { Authentication, Fleet } from "@formant/data-sdk";
import "../../../css/formant.css";
import "./style.css";
import { definedAndNotNull } from "../../../packages/common/defined";
import "../../../packages/ui-sdk-joystick/src/main";
import "../../../packages/ui-sdk-realtime-player/src/main";

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
const elForm = definedAndNotNull(document.querySelector("form")) as HTMLElement;

function log(msg: string) {
  elLog.innerHTML = msg + "<br>" + elLog.innerHTML;
}

elBtnConnect.addEventListener("click", async () => {
  elForm.style.display = "none";
  elLog.style.display = "block";
  elPlayer.style.display = "block";
  elJoystick.style.display = "block";
  log("Connecting ...");
  elJoystick.addEventListener("joystick", (e) => {
    log(JSON.stringify(e.detail));
  });
});
