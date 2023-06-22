import { Authentication, Fleet } from "../src/main";
import "./style.css";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

(async function () {
  const el = document.querySelector("#app");

  if (el) {
    try {
      await Authentication.waitTilAuthenticated();
      const a = await Fleet.getFleetDevices(
        "03c35af7-e82b-44f0-997e-f6d564180c18"
      );

      console.log(a);
    } catch (error) {
      console.log(error);
    }
  }
})();
