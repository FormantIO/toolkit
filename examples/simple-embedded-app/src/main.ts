import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";

async function main() {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";

    debugger;
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();
      el.innerHTML = JSON.stringify(await device.getLatestTelemetry());
    } else {
      el.innerHTML = "Not Authenticated";
    }
  }
}

(async () => {
  try {
    await main();
  } catch (e) {
    console.log(e);
  }
})();
