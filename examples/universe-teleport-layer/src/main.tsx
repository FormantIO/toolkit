import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { LayerRegistry, Universe } from "@formant/universe";
import { SimulatedUniverseData } from "../../../packages/universe/demo/SimulatedUniverseData";
import * as uuid from "uuid";
import { TeleportLayer } from "./TeleportLayer";

LayerRegistry.register(TeleportLayer);

function App() {
  return (
    <Universe
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
          type: "teleport",
          name: "Teleport",
          deviceContext: undefined,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0.5 },
          fieldValues: {},
          data: {},
        },
      ]}
      universeData={new SimulatedUniverseData()}
      mode="edit"
      vr
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
