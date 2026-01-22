// Import polyfills first
import "./polyfills";
import { App, Fleet, Authentication } from "@formant/data-sdk";

type IDevice = {
  id: string;
  name: string;
  tags?: Record<string, string>;
};

type DeviceStreamData = {
  deviceId: string;
  name: string;
  points: any[];
};

type Stream = {
  data: DeviceStreamData[];
};

type ModuleData = {
  streams: Record<string, Stream>;
};

if (App.isModule()) {
  App.showMessage("This running as a module");
} else {
  window.alert("This is not running as a module");
}

(document.querySelector("#sendmessage") as HTMLElement).addEventListener(
  "click",
  () => {
    App.showMessage("hello world " + Math.random());
  }
);

(document.querySelector("#refreshtoken") as HTMLElement).addEventListener(
  "click",
  () => {
    App.refreshAuthToken();
  }
);

(document.querySelector("#requestmoduledata") as HTMLElement).addEventListener(
  "click",
  () => {
    App.requestModuleData();
  }
);

(document.querySelector("#goback") as HTMLElement).addEventListener(
  "click",
  () => {
    App.showMessage("moving timeline back one minute");
    App.goToTime(new Date(new Date().getTime() - 60 * 1000));
  }
);

(document.querySelector("#gotodevice") as HTMLElement).addEventListener(
  "click",
  () => {
    App.showMessage("going to device");
    App.goToDevice("9309992b-452b-42fc-9aa2-dbd7e66a90cf");
  }
);

(document.querySelector("#fetchcurrent") as HTMLElement).addEventListener(
  "click",
  async () => {
    try {
      const data = await Fleet.getOnlineDevices();
      App.showMessage("online devices: " + JSON.stringify(data));
    } catch (error: any) {
      console.error("Error fetching online devices:", error);
      const errorMsg = error.message || String(error);
      if (errorMsg.includes("Not authenticated") || errorMsg.includes("authenticated")) {
        App.showMessage(
          "Note: This requires real API authentication.\n" +
          "In production, the host provides a valid auth token.\n" +
          "Error: " + errorMsg
        );
      } else {
        App.showMessage("Error: " + errorMsg);
      }
    }
  }
);

(document.querySelector("#sendchannel") as HTMLElement).addEventListener(
  "click",
  () => {
    App.sendChannelData("test_channel", { abc: 123 });
  }
);

(document.querySelector("#testgroupdevices") as HTMLElement).addEventListener(
  "click",
  async () => {
    try {
      // Check if the API exists (for SDK version compatibility)
      if (typeof Fleet.getGroupDevices !== "function") {
        App.showMessage(
          "Error: Fleet.getGroupDevices() is not available.\n" +
          "This requires SDK version with group device support.\n" +
          "Current SDK version may not include this feature yet."
        );
        console.error("Fleet.getGroupDevices is not a function");
        return;
      }

      const groupDevices = Fleet.getGroupDevices() as IDevice[] | undefined;
      if (groupDevices) {
        const deviceList = groupDevices
          .map((d: IDevice) => `- ${d.name} (${d.id})`)
          .join("\n");
        
        App.showMessage(
          `✅ Group devices detected: ${groupDevices.length} devices\n${deviceList}`
        );
        console.log("Group devices (IDevice[]):", groupDevices);
        
        // Test getting full Device instances (optional - requires auth)
        if (typeof Fleet.getGroupDevicesAsDeviceInstances === "function") {
          try {
            const deviceInstances = await Fleet.getGroupDevicesAsDeviceInstances();
            console.log("Group devices (Device[]):", deviceInstances);
            App.showMessage(
              `✅ Full Device instances: ${deviceInstances.length} devices loaded`
            );
          } catch (err: any) {
            console.error("Error getting device instances:", err);
            const errorMsg = err.message || String(err);
            if (errorMsg.includes("Not authenticated") || errorMsg.includes("authenticated")) {
              App.showMessage(
                `✅ Group devices (IDevice[]): ${groupDevices.length} devices\n${deviceList}\n\n` +
                `Note: Full Device instances require API authentication.\n` +
                `The basic group devices info above works without auth.`
              );
            } else {
              App.showMessage("Error loading device instances: " + errorMsg);
            }
          }
        } else {
          App.showMessage(
            `✅ Group devices: ${groupDevices.length} devices\n${deviceList}\n\n` +
            `Note: getGroupDevicesAsDeviceInstances() not available in this SDK version`
          );
        }
      } else {
        App.showMessage("No group devices found (single device context or devices not loaded yet)");
        console.log("Fleet.getGroupDevices():", groupDevices);
        console.log("Tip: Make sure host sent overview_devices with multiple devices");
      }
    } catch (error: any) {
      console.error("Error testing group devices:", error);
      App.showMessage("Error: " + (error.message || String(error)));
    }
  }
);

// API-based group lookup (reads ?group= from URL). Requires real auth.
(document.querySelector("#fetchcurrentgroup") as HTMLElement).addEventListener(
  "click",
  async () => {
    try {
      if (typeof Fleet.getCurrentGroup !== "function") {
        App.showMessage(
          "Error: Fleet.getCurrentGroup() is not available in this SDK version."
        );
        return;
      }

      const devices = await Fleet.getCurrentGroup();
      if (!devices) {
        App.showMessage("No ?group= in URL (host group field is empty)");
        return;
      }

      App.showMessage(`✅ getCurrentGroup(): ${devices.length} devices`);
      console.log("getCurrentGroup() devices:", devices);
    } catch (error: any) {
      console.error("Error calling getCurrentGroup():", error);
      const errorMsg = error?.message || String(error);
      if (errorMsg.includes("Not authenticated") || errorMsg.includes("authenticated")) {
        App.showMessage(
          "Note: Fleet.getCurrentGroup() requires real API authentication.\n" +
            "In production, the host provides a valid auth token.\n" +
            "Error: " +
            errorMsg
        );
      } else {
        App.showMessage("Error: " + errorMsg);
      }
    }
  }
);

App.addChannelDataListener(
  "test_channel",
  (e: { source: string; data: any }) => {
  if (e.source === App.getCurrentModuleContext()) {
    App.showMessage("channel data i sent: " + JSON.stringify(e));
  } else {
    App.showMessage("channel data: " + JSON.stringify(e));
  }
  }
);

App.setupModuleMenus([
  {
    label: "My Menu Item",
  },
]);

App.addMenuListener((label: string) => {
  App.showMessage(label + " was clicked!");
});

App.addAccessTokenRefreshListener((token: string) => {
  App.showMessage("token refreshed " + token);
});

App.addModuleDataListener((data: ModuleData) => {
  console.log("recieved data", data);
  // Example: Process multi-device stream data
  Object.entries(data.streams).forEach(([streamName, stream]) => {
    console.log(`Stream: ${streamName}, Devices: ${stream.data.length}`);
    stream.data.forEach((deviceData: DeviceStreamData) => {
      console.log(
        `  Device ${deviceData.deviceId} (${deviceData.name}): ${deviceData.points.length} points`
      );
    });
  });
});

// Group device awareness example
try {
  if (typeof App.addOverviewDeviceListener === "function") {
    App.addOverviewDeviceListener((devices: IDevice[]) => {
      console.log("Overview devices received:", devices);
      if (typeof Fleet.getGroupDevices === "function") {
        const groupDevices = Fleet.getGroupDevices() as IDevice[] | undefined;
        if (groupDevices && groupDevices.length > 1) {
          App.showMessage(
            `Group context: ${groupDevices.length} devices available`
          );
        } else if (groupDevices && groupDevices.length === 1) {
          App.showMessage("Single device context");
        }
      } else {
        console.log("Fleet.getGroupDevices() not available in this SDK version");
      }
    });
  } else {
    console.warn("App.addOverviewDeviceListener() not available in this SDK version");
  }
} catch (error) {
  console.error("Error setting up overview device listener:", error);
}

(async function () {
  await Authentication.waitTilAuthenticated();
  const config = await App.getCurrentModuleConfiguration();
  const el = document.querySelector("#config") as HTMLElement;
  if (config) {
    el.innerText = config;
    el.style.display = "block";
  } else {
    // No configuration is normal - modules can work without it
    el.style.display = "none";
  }
  App.addModuleConfigurationListener((event: { configuration: string }) => {
    const el = document.querySelector("#config") as HTMLElement;
    if (event.configuration) {
      el.innerText = event.configuration;
      el.style.display = "block";
    } else {
      el.style.display = "none";
    }
  });
})();
