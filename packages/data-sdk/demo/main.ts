import { Authentication, App } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();

      App.addOverviewDeviceListener((_) => console.log(_));

      // console.log(l);
    } catch (error) {
      console.log(error);
    }
  }
})();
