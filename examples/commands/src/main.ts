import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";

if (!(await Authentication.waitTilAuthenticated())) {
  log("Not Authenticated");
}

// get current device from url context
const device = await Fleet.getCurrentDevice();

el("#commands").innerHTML = (await device.getAvailableCommands())
  .map((_) => _.command)
  .join(", ");

function log(msg: string) {
  el("#log").innerHTML = msg + "<br>" + el("#log").innerHTML;
}

function el(selector: string) {
  return document.querySelector(selector) as HTMLElement;
}

el("button").addEventListener("click", async () => {
  try {
    await device.sendCommand(
      (el("#command_name") as HTMLInputElement).value,
      (el("#command_data") as HTMLInputElement).value
    );
  } catch (e: any) {
    log(e.toString());
    return;
  }
  log("Command sent!");
});

document.body.style.display = "block";
