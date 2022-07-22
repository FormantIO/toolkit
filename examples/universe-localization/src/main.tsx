import * as React from "react";
import { createRoot } from "react-dom/client";
import {
  Button,
  FormantProvider,
  Stack,
  TextField,
  Typography,
} from "@formant/ui-sdk";
import styled from "styled-components";
import { Authentication, Device, Fleet } from "@formant/data-sdk";
import { Universe } from "@formant/universe";
import * as uuid from "uuid";
import { LiveUniverseData } from "@formant/universe-connector";

function App({ deviceId }: { deviceId: string }) {
  return (
    <Universe
      initialSceneGraph={[
        {
          id: uuid.v4(),
          editing: false,
          type: "ground",
          name: "Ground",
          deviceContext: deviceId,
          children: [],
          visible: true,
          position: { type: "manual", x: 0, y: 0, z: 0 },
          fieldValues: {
            flatAxes: {
              type: "boolean",
              value: true,
            },
          },
          data: {},
        },
      ]}
      universeData={new LiveUniverseData()}
      mode="view"
      vr
    />
  );
}

const Centered = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  height: 100vh;
`;

function Login() {
  const [loggedIn, setLoggedIn] = React.useState(false); // false
  const [username, setUsername] = React.useState(import.meta.env.VITE_EMAIL);
  const [password, setPassword] = React.useState(import.meta.env.VITE_PWD);
  const [devices, setDevices] = React.useState<Device[]>([]);
  const [deviceId, setDeviceId] = React.useState<string | undefined>(undefined);
  return loggedIn && deviceId ? (
    <App deviceId={deviceId} />
  ) : (
    <Centered>
      {loggedIn ? (
        <div>
          {devices.map((_) => (
            <Button
              key={_.id}
              variant="contained"
              size="small"
              type="button"
              onClick={async () => {
                setDeviceId(_.id);
              }}
            >
              {_.name}
            </Button>
          ))}
        </div>
      ) : (
        <Stack gap={3}>
          <Typography variant="h1">Login</Typography>
          <div>
            <TextField
              label="email"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
            />
          </div>
          <div>
            <TextField
              label="password"
              type="password"
              value={"password"}
              onChange={(e) => setPassword(e.target.value)}
            />
          </div>
          <Button
            variant="contained"
            size="large"
            type="button"
            onClick={async () => {
              await Authentication.login(username, password);
              setLoggedIn(true);
              const devices = await Fleet.getDevices();
              setDevices(devices);
            }}
          >
            Login
          </Button>
        </Stack>
      )}
    </Centered>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider>
      <Login />
    </FormantProvider>
  );
}
