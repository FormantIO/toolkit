import { Authentication, Fleet } from "../src/main";
import "./style.css";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();

      el.innerHTML =
        "Starting realtikme connection for " + JSON.stringify(device);
      await device.startRealtimeConnection();
      el.innerHTML = "Realtime Connected";
      device.addRealtimeListener((_peer, msg) => {
        console.log(msg);
      });
      const videoStreams = await device.getRealtimeVideoStreams();

      el.innerHTML = JSON.stringify(videoStreams);
      console.log("starting stream");
      await device.startListeningToRealtimeVideo(videoStreams[0]);
      el.innerHTML = "Authenticated";
    } else {
      el.innerHTML = "Not Authenticated";
    }
  }
})();
