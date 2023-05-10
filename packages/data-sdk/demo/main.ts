import { Authentication, App, Fleet, SessionType } from "../src/main";
import "./style.css";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();
      const d = await Fleet.getCurrentDevice();
      let conected = false;
      while (!conected) {
        console.warn("waiitng for main connection");
        conected = await d.isInRealtimeSession();
        await timeout(2000);
      }
      console.warn("star onnection");
      await d.startRealtimeConnection(SessionType.Observe);
    } catch (error) {
      console.log(error);
    }
  }
})();
