import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";

const el = document.querySelector("#app");
if (el) {
  el.innerHTML = "Connecting";
  if (await Authentication.waitTilAuthenticated()) {
    const devices = await Fleet.getDevices();
    el.innerHTML =
      Authentication.getCurrentUser()?.firstName +
      " is Authenticated, here's a list of known robots:<br/>" +
      devices.map((_) => _.name).join("<br>");
  } else {
    el.innerHTML = "Not Authenticated";
  }
}
