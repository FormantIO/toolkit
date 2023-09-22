import * as React from "react";
import { createRoot } from "react-dom/client";
import { FormantProvider, StreamLoader } from "../src/main";

import styled from "@emotion/styled";

function App() {
  const [auth, setAuth] = React.useState(true);

  return (
    <div>
      {!auth ? (
        <></>
      ) : (
        <Container>
          <StreamLoader
            streams={["$.host.cpu"]}
            types={["numeric set"]}
            deviceIds={[
              "8b1b3e3e-1c41-4820-99ca-97657374e2c7",
              "58d7f6e1-899d-4a8a-8c02-4c805cc8227f",
            ]}
            useCurrentDevice={false}
          >
            <></>
          </StreamLoader>
        </Container>
      )}
    </div>
  );
}

const Container = styled.div`
  height: 100vh;
  width: 100%;
  display: flex;
  flex-wrap: wrap;
  overflow: hidden;
  background-color: black;
`;

const Cell = styled.div`
  height: 50vh;
  width: 50vw;
`;

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider parseConfiguration>
      <App />
    </FormantProvider>
  );
}
