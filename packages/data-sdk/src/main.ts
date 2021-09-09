export { Fleet } from "./Fleet";
export { Authentication } from "./Authentication";
export { Device } from "./Device";

import { Fleet } from "./Fleet";
import { Authentication } from "./Authentication";

export const FORMANT_API_URL = "https://api.formant.io";

const urlParams = new URLSearchParams(window.location.search);

const urlDevice = urlParams.get("device");
if (urlDevice) {
  Fleet.setDefaultDevice(urlDevice);
}

const urlAuth = urlParams.get("auth");
if (urlAuth) {
  Authentication.loginWithToken(urlAuth);
}
