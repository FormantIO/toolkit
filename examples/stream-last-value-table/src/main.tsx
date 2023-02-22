import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import "./index.css";
import { FormantProvider } from "@formant/ui-sdk";
import { RecoilRoot } from "recoil";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <FormantProvider parseConfiguration >
    <App />
  </FormantProvider>
);
