import { Authentication, Fleet } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");
  if (el) {
    if (await Authentication.waitTilAuthenticated()) {
      const views = await Fleet.getViews();

      const fg = views.filter((_) => _.name === "FoxGlove")[0];
      const r = await Fleet.patchView({ ...fg, url: `${fg.url}/?auth={auth}` });
      console.log(r);
    }
  }
})();
