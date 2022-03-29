import React from "react";
import ReactDOM from "react-dom";
import "./index.css";
import App from "./App";
import { Formant } from "@alenjdev/ui-sdk";

ReactDOM.render(
  <Formant>
    <App />
  </Formant>,
  document.getElementById("root")
);
