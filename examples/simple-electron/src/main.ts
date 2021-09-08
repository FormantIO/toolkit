import { Authentication, Fleet } from "@formant/data-sdk";
import "../../../css/formant.css";

document.querySelector("button")?.addEventListener("click", async () => {
  const el = document.querySelector("#app");
  const usernameField = document.querySelector("#username") as HTMLInputElement;
  const passwordField = document.querySelector("#password") as HTMLInputElement;
  if (el && usernameField && passwordField) {
    el.innerHTML = "Connecting";
    try {
      await Authentication.login(usernameField.value, passwordField.value);
      const devices = await Fleet.getDevices();
      el.innerHTML =
        Authentication.getCurrentUser()?.firstName +
        " is Authenticated, here's a list of known robots:<br/>" +
        devices.map((_) => _.name).join("<br>");
    } catch (e: any) {
      el.innerHTML = e.toString();
    }
  }
});
