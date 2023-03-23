import { Authentication, Fleet } from "../src/main";
import "./style.css";
import { IAnnotationQuery } from "../src/model/IAnnotationQuery";
(async function () {
  const el = document.querySelector("#app");
  if (el) {
    const seconds = 1000;
    const minutes = 60 * seconds;
    const hours = 60 * minutes;
    const days = 24 * hours;

    if (await Authentication.waitTilAuthenticated()) {
      const views = await Fleet.getViews();

      const fg = views.filter((_) => _.name === "FoxGlove")[0];
      const r = await Fleet.patchView({ ...fg, url: `${fg.url}/?auth={auth}` });
      console.log(r);
    }
  }
})();
