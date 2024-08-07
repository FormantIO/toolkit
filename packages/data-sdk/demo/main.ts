import { Authentication, Fleet, SessionType } from "../src/main";
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
      let connected = false;
      while (!connected) {
        console.warn("waiting for main connection");
        connected = await d.isInRealtimeSession();
        await timeout(2000);
      }
      console.warn("start connection");
      await d.startRealtimeConnection(SessionType.Observe);
      console.error("complete");
    } catch (error) {
      console.log(error);
    }
  }
})();
