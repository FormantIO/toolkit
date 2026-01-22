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

    // Auto-initialize group devices from overview_devices message
    // This enables modules to work in coherence group views
    // 
    // Priority order (when devices are available):
    // 1. overview_devices message (from host) - highest priority, overrides URL device
    //    - Arrives asynchronously via postMessage
    //    - Automatically calls Fleet.setGroupDevices() when received
    // 2. URL ?device= parameter - set synchronously at init
    //    - May be overridden by overview_devices if it arrives later
    // 3. URL ?group= parameter - requires explicit call to Fleet.getCurrentGroup()
    //    - NOT called automatically - modules must call it explicitly if needed
    //    - Requires authentication
    App.addOverviewDeviceListener((devices) => {
      if (devices && devices.length > 0) {
        Fleet.setGroupDevices(devices);
      }
    });
  }

  if (typeof window !== "undefined") {
    App.listenForConnectionEvents();
  }
} catch (_) {}
