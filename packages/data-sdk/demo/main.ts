import { Authentication, Fleet } from "../src/main";
import "./style.css";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();
      const streams = await device.getTelemetryStreams();

      streams.forEach(async (_) => {
        console.log(
          await device.getTelemetry(
            _.name,
            (() => {
              var dt = new Date();
              dt.setHours(dt.getHours() - 1);
              return dt;
            })(),
            new Date()
          )
        );
      });
    }
  }
})();
