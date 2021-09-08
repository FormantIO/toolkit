import { DataManager } from "../src/DataManager";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");
  if (el) {
    el.innerHTML = "Connecting";
    if (await DataManager.waitTilAuthenticated()) {
      el.innerHTML =
        DataManager.getCurrentUser()?.firstName + " is Authenticated";
    } else {
      el.innerHTML = "Not Authenticated";
    }
  }
})();
