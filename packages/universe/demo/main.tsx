import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { Universe } from "../src/main";
import { SimulatedUniverseData } from "./SimulatedUniverseData";
import { createScene } from "./createScene";

function App() {
  return (
    <Universe
      initialSceneGraph={createScene()}
      universeData={new SimulatedUniverseData()}
      mode="edit"
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
