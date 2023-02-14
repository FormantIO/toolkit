import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import "./index.css";
import { FormantProvider } from "@formant/ui-sdk";
import { store } from "./app/store";
import { Provider } from "react-redux";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <Provider store={store}>
    <FormantProvider parseConfiguration>
      <React.StrictMode>
        <App />
      </React.StrictMode>
    </FormantProvider>
  </Provider>
);
