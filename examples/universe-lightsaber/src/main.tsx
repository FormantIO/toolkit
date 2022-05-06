import * as React from "react";
import { createRoot } from "react-dom/client";
import * as uuid from "uuid";
import { FormantProvider } from "@formant/ui-sdk";
import { LayerRegistry, Universe } from "@formant/universe";
import { SimulatedUniverseData } from "../../../packages/universe/demo/SimulatedUniverseData";
import { LightsaberLayer } from "./LightsaberLayer";

LayerRegistry.register(LightsaberLayer);

function App() {
  return (
    <Universe
      universeData={new SimulatedUniverseData()}
      initialSceneGraph={[
        {
          id: uuid.v4(),
          editing: false,
          type: "ground",
          name: "Ground",
          deviceContext: undefined,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {},
          data: {},
        },
        {
          id: uuid.v4(),
          editing: false,
          type: "lightsaber",
          name: "Lightsaber",
          deviceContext: undefined,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {},
          data: {},
        },
      ]}
      mode={"view"}
      vr={true}
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
