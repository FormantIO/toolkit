import { Authentication, Fleet } from "../src/main";
import "./style.css";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    if (await Authentication.waitTilAuthenticated()) {
      el.innerHTML = "Authenticated";
      const device = await Fleet.getCurrentDevice();
      el.innerHTML = "Connected";
    } else {
      el.innerHTML = "Not Authenticated";
    }
  }
})();
