import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { LayerRegistry, TeleportLayer, Universe } from "../src/main";
import { SimulatedUniverseData } from "./SimulatedUniverseData";
import { createScene } from "./createScene";
import { XboxLayer } from "./XboxLayer";

LayerRegistry.register(TeleportLayer);
LayerRegistry.register(XboxLayer);

function App() {
  const data = new SimulatedUniverseData();
  window.setInterval(() => {
    data.setTime(Date.now());
  }, 60 / 12);
  return (
    <Universe
      initialSceneGraph={createScene()}
      universeData={data}
      mode="edit"
      vr={true}
      onSceneGraphChange={(_) => {
        console.log(JSON.stringify(_));
      }}
    />
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
