import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { Universe, SimulatedUniverseData } from "../src/main";

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
