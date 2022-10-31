import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import "./index.css";
import { FormantProvider } from "@formant/ui-sdk";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <FormantProvider>
    <React.StrictMode>
      <App />
    </React.StrictMode>
  </FormantProvider>
);
