import { Authentication, Fleet } from "../src/main";
import "./style.css";
import { deserializeHash } from "../src/main";

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();
      const d = await Fleet.getCurrentDevice();

      const l = await d.createShareLink(
        {
          scope: {
            start: new Date(Date.now() - 1000 * 60 * 30).toISOString(),
            end: new Date(Date.now() + 1000 * 60 * 30).toISOString(),
          },
          time: new Date(Date.now() - 1000 * 60 * 10).toISOString(),
        },
        "Dev"
      );

      console.log(l);
    } catch (error) {
      console.log(error);
    }
  }
})();
