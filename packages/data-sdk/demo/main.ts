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

    const query: IAnnotationQuery = {
      start: new Date(Date.now() - 90 * days).toISOString(),
      end: new Date(Date.now()).toISOString(),
    };
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();
      const annotations = await device.getAnnotationCountByIntervals(
        query,
        "Alen ESR",
        "week"
      );
      console.log(annotations);
    }
  }
})();
