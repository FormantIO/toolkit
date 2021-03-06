import "@formant/ui-sdk-login";
import { Authentication, Fleet } from "@formant/data-sdk";
import "./style.css";

const l = document.querySelector("formant-login") as HTMLElement;
l.addEventListener("login", async (e) => {
  l.setAttribute("message", "Connecting ...");
  l.setAttribute("error", "");
  const creds = e as CustomEvent<{ username: string; password: string }>;
  try {
    await Authentication.login(creds.detail.username, creds.detail.password);
  } catch (e) {
    l.setAttribute("message", "");
    l.setAttribute("error", "Failed to login");
    return;
  }
  l.remove();
  document.body.innerHTML = JSON.stringify(
    (await Fleet.getDevices()).map((_) => _.name)
  );
});
