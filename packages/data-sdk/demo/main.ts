import { Authentication } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");
  if (el) {
    try {
      const t = await Authentication.refresh("");
      console.log(
        t,
        Authentication.token,
        Authentication.refreshToken,
        Authentication.currentUser
      );
    } catch (error) {
      console.log(error);
    }
  }
})();
