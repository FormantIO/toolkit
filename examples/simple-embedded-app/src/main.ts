import { DataManager } from "@formant/data-manager";
import "./style.css";

const el = document.querySelector("#app");
if (el) {
  el.innerHTML = "Connecting";
  if (await DataManager.waitTilAuthenticated()) {
    const devices = await DataManager.getDevices();
    el.innerHTML =
      DataManager.getCurrentUser()?.firstName +
      " is Authenticated, here's a list of known robots:<br/>" +
      devices.map((_) => _.name).join("<br>");
  } else {
    el.innerHTML = "Not Authenticated";
  }
}
