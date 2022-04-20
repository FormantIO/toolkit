import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import {
  LayerRegistry,
  Universe,
  SimulatedUniverseData,
} from "@formant/universe";

import { CubeLayer } from "./CubeLayer";

LayerRegistry.register(CubeLayer);

function App() {
  return (
    <div>
      <Universe universeData={new SimulatedUniverseData()}></Universe>
    </div>
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
