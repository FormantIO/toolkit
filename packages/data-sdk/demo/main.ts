import { Authentication, Fleet } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();
      const c = await Fleet.getCurrentDevice();
      console.log(await c.getRealtimeVideoStreams());

      // console.log(l);
    } catch (error) {
      console.log(error);
    }
  }
})();
