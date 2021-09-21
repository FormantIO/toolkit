import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";
import "../../../packages/ui-sdk-joystick/src/main";
import "../../../packages/ui-sdk-realtime-player/src/main";

const elBtnConnect = document.querySelector("button") as HTMLElement;
const elLog = document.querySelector("#log") as HTMLElement;
const elPlayer = document.querySelector(
  "formant-realtime-player"
) as HTMLElement;
const elJoystick = document.querySelector("formant-joystick") as HTMLElement;
const elSection = document.querySelector("section") as HTMLElement;

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

  log("Currently looking at <b>" + device.name + "</b>");
  log("Getting a realtime connection ... ");
  await device.startRealtimeConnection();
  log("Connected");

  // show the player
  elPlayer.style.display = "block";

  // show joysticks and connec them up
  elJoystick.style.display = "block";
  elJoystick.addEventListener("joystick", (e) => {
    const ce = e as CustomEvent;
    log(JSON.stringify(ce.detail));
  });
});

document.body.style.display = "block";
