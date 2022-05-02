import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { Universe } from "@formant/universe";
import { SimulatedUniverseData } from "./SimulatedUniverseData";
import { createScene } from "./createScene";

const queryString = window.location.search;
const urlParams = new URLSearchParams(queryString);
const mode = urlParams.get("mode");
const vr = urlParams.get("mode");

function App() {
  return (
    <Universe
      universeData={new SimulatedUniverseData()}
      mode={
        mode === "edit" || mode === "view" || mode === "no-interaction"
          ? mode
          : "edit"
      }
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
