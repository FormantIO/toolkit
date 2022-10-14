import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import "./index.css";
import { FormantProvider } from "@formant/ui-sdk";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <FormantProvider>
    {/* <React.StrictMode> */}
    <App />
    {/* </React.StrictMode> */}
  </FormantProvider>
);

// import { Authentication, Fleet } from "../src/main";
// import "./style.css";
// (async function () {
//   const el = document.querySelector("#app");
//   if (el) {
//     if (await Authentication.waitTilAuthenticated()) {
//       const events = await Fleet.queryEvents({});
//       console.log(events);
//       const data = await Fleet.queryTelemetry({
//         start: new Date(Date.now() - 1000 * 60).toISOString(),
//         end: new Date().toISOString(),
//         deviceIds: Fleet.defaultDeviceId ? [Fleet.defaultDeviceId] : [],
//       });
//       console.log(data);
//     }
//   }
// })();
