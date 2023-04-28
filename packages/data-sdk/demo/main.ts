import { Authentication, App } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();
      const b = document.getElementById("btn");
      b?.addEventListener("click", async () => {
        const d = await App.getDate(
          undefined,
          new Date("Sat Apr 20 2023 00:00:00 GMT-0500 (Central Daylight Time)")
        );
        console.log(d);
      });
    } catch (error) {
      console.log(error);
    }
  }
})();
