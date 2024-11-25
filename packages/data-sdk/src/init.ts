// Application Initialization
import { App } from "./App";
import { Authentication } from "./Authentication";
import { Fleet } from "./Fleet";

try {
  const urlParams =
    typeof window !== "undefined" && window.location
      ? new URLSearchParams(window.location.search)
      : new URLSearchParams("");

  const urlDevice = urlParams.get("device");
  if (urlDevice) {
    Fleet.setDefaultDevice(urlDevice);
  }

  const urlAuth = urlParams.get("auth");
  if (urlAuth) {
    Authentication.loginWithToken(urlAuth);
  }

  const moduleName = urlParams.get("module");
  if (moduleName) {
    Authentication.listenForRefresh();
  }

  if (typeof window !== "undefined") {
    App.listenForConnectionEvents();
  }
} catch (_) {}
