export { Fleet } from "./Fleet";
export * from "./Authentication";
export * from "./Device";
export * from "./DataChannel";

import { Fleet } from "./Fleet";
import { Authentication } from "./Authentication";

const urlParams = new URLSearchParams(window.location.search);

const urlDevice = urlParams.get("device");
if (urlDevice) {
  Fleet.setDefaultDevice(urlDevice);
}

const urlAuth = urlParams.get("auth");
if (urlAuth) {
  Authentication.loginWithToken(urlAuth);
}
