import { Authentication, Fleet } from "../src/main";
import "./style.css";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();
      await device.startRealtimeConnection();
      const manipulators = await device.getRealtimeManipulators();
      console.log(manipulators);
      //await manipulators[0].synchronize();
    }
  }
})();
