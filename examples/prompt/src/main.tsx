import React, { useEffect } from "react";
import ReactDOM from "react-dom/client";
import "./index.css";
import { Authentication, App } from "@formant/data-sdk";

const Demo = () => {
  const [result, setResult] = React.useState<any>(null);
  return (
    <>
      <button
        onClick={() => {
          (async () =>
            setResult(
              await App.prompt({
                title: "Hello",
                type: "object",
                properties: {
                  name: {
                    type: "string",
                    title: "Name",
                  },
                },
              })
            ))();
        }}
      >
        Click me
      </button>
      {result && <pre>{JSON.stringify(result, null, 2)}</pre>}
    </>
  );
};

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    <Demo />
  </React.StrictMode>
);
