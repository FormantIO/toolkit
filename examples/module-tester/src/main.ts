import { App, ModuleData, Fleet, Authentication } from "@formant/data-sdk";

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
  () => {
    Fleet.getOnlineDevices().then((data: any) => {
      App.showMessage("online devices: " + JSON.stringify(data));
    });
  }
);

(document.querySelector("#sendchannel") as HTMLElement).addEventListener(
  "click",
  () => {
    App.sendChannelData("test_channel", { abc: 123 });
  }
);

App.addChannelDataListener("test_channel", (e) => {
  if (e.source === App.getCurrentModuleContext()) {
    App.showMessage("channel data i sent: " + JSON.stringify(e));
  } else {
    App.showMessage("channel data: " + JSON.stringify(e));
  }
});

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
});

(async function () {
  await Authentication.waitTilAuthenticated();
  const config = await App.getCurrentModuleConfiguration();
  const el = document.querySelector("#config") as HTMLElement;
  el.innerText = config ? config : "no configuration found";
  App.addModuleConfigurationListener((event) => {
    const el = document.querySelector("#config") as HTMLElement;
    el.innerText = event.configuration;
  });
})();
