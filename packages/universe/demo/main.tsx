import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider } from "@formant/ui-sdk";
import { Universe, SimulatedUniverseData } from "../src/main";

function App() {
  return (
    <Universe universeData={new SimulatedUniverseData()} mode="edit"></Universe>
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
