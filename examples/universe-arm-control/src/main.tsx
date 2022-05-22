import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { LayerRegistry, TeleportLayer, Universe } from "@formant/universe";
import { SimulatedUniverseData } from "./SimulatedUniverseData";
import { createScene } from "./createScene";
import { App as FormantApp, ModuleData } from "@formant/data-sdk";
import { HandsLayer } from "./HandsLayer";
import { XboxLayer } from "./XboxLayer";

const queryString = window.location.search;
const urlParams = new URLSearchParams(queryString);
const vr = urlParams.get("vr");

LayerRegistry.register(TeleportLayer);
LayerRegistry.register(HandsLayer);
LayerRegistry.register(XboxLayer);

function App() {
  const data = new SimulatedUniverseData();
  if (FormantApp.isModule()) {
    FormantApp.addModuleDataListener((moduleData: ModuleData) => {
      data.setTime(moduleData.time);
    });
  } else {
    window.setInterval(() => {
      data.setTime(Date.now());
    }, 60 / 12);
  }
  return (
    <Universe
      universeData={data}
      mode={"view"}
      initialSceneGraph={createScene()}
      onSceneGraphChange={(_) => {
        console.log(JSON.stringify(_));
      }}
      vr={vr === "true"}
    ></Universe>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider>
      <App />
    </FormantProvider>
  );
}
