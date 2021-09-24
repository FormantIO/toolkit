import { Authentication, Fleet } from "../src/main";
import "./style.css";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    await Authentication.login("ab", "abc");
    if (await Authentication.waitTilAuthenticated()) {
      el.innerHTML = "Authenticated";
      const device = await Fleet.getCurrentDevice();
      await device.startRealtimeConnection();
      const j = await device.createCustomDataChannel("joystick");
      el.innerHTML = "Data channel created";
      await j.waitTilReady();
      el.innerHTML = "Data channel ready!";
      j.send("konami");
      j.addListener((message) => {
        console.log(message);
      });
      el.innerHTML = "Sent";
    } else {
      el.innerHTML = "Not Authenticated";
    }
  }
})();
