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
          (async () => {
            const r = await App.prompt(
              {
                title: "Send to location",
                description:
                  "This is a description of some action this prompt is going to do",
                type: "object",
                "$formant.documentationUrl":
                  "https://www.youtube.com/watch?v=jfKfPfyJRdk",
                properties: {
                  speed: {
                    type: "string",
                    title: "How fast?",
                    enum: ["slow", "fast"],
                  },
                  afterWards: {
                    type: "string",
                    title: "What should the device do after?",
                    enum: ["stop", "return to home base"],
                  },
                },
              },
              {
                okText: "Go To Location",
                cancelText: "Do nothing",
              }
            );
            setResult(r);
          })();
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
