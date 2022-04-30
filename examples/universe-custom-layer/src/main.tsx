import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import {
  LayerRegistry,
  Universe,
  SimulatedUniverseData,
} from "@formant/universe";

const queryString = window.location.search;
const urlParams = new URLSearchParams(queryString);
const mode = urlParams.get("mode");
const vr = urlParams.get("mode");

import { CubeLayer } from "./CubeLayer";

LayerRegistry.register(CubeLayer);

function App() {
  return (
    <Universe
      universeData={new SimulatedUniverseData()}
      mode={
        mode === "edit" || mode === "view" || mode === "no-interaction"
          ? mode
          : "edit"
      }
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
