import { DataManager } from "@formant/data-manager";
import "../../../css/formant.css";

document.querySelector("button")?.addEventListener("click", async () => {
  debugger;
  const el = document.querySelector("#app");
  const usernameField = document.querySelector("#username") as HTMLInputElement;
  const passwordField = document.querySelector("#password") as HTMLInputElement;
  if (el && usernameField && passwordField) {
    el.innerHTML = "Connecting";
    try {
      await DataManager.login(usernameField.value, passwordField.value);
      const devices = await DataManager.getDevices();
      el.innerHTML =
        DataManager.getCurrentUser()?.firstName +
        " is Authenticated, here's a list of known robots:<br/>" +
        devices.map((_) => _.name).join("<br>");
    } catch (e: any) {
      el.innerHTML = e.toString();
    }
  }
});
