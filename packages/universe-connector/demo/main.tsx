import React, { useEffect, useState } from "react";
import ReactDOM from "react-dom/client";

import { Authentication, Fleet } from "@formant/data-sdk";
import { defined } from "../src/main";

export function App() {
  const [loggedIn, setLoggedIn] = useState(false);
  const [allDevices, setAllDevices] = useState<
    | {
        name: string;
        id: string;
      }[]
    | null
  >([]);
  const [currentDevice, setCurrentDevice] = useState<{
    name: string;
    id: string;
  } | null>(null);

  useEffect(() => {
    (async () => {
      if (currentDevice) {
        const d = await Fleet.getDevice(currentDevice.id);
        const conf = await defined(d).getConfiguration();
        const teleopConf = conf.teleop;
        const customStreams = defined(teleopConf).customStreams;
        const bitsets =
          customStreams?.filter((_) => _.rtcStreamType === "bitset") || [];
        console.log(bitsets);
        await d.startRealtimeConnection();
        d.startListeningToRealtimeDataStream({
          name: "Status",
        });
        d.addRealtimeListener((data) => {
          console.log(data);
        });
      }
    })();
  }, [currentDevice]);

  if (!loggedIn) {
    return (
      <div>
        <div>
          <input type="text" id="username" placeholder="username" />
        </div>
        <div>
          <input type="password" id="password" placeholder="password" />
        </div>
        <div>
          <button
            onClick={() => {
              (async () => {
                const username = document.getElementById(
                  "username"
                ) as HTMLInputElement;
                const password = document.getElementById(
                  "password"
                ) as HTMLInputElement;
                if (username && password) {
                  await Authentication.login(username.value, password.value);
                  setLoggedIn(true);
                  const devices = await Fleet.getOnlineDevices();

                  if (devices.length > 0) {
                    setAllDevices(
                      devices.map((device) => {
                        return {
                          name: device.name,
                          id: device.id,
                        };
                      })
                    );
                  }
                }
              })();
            }}
          >
            Login
          </button>
        </div>
      </div>
    );
  }

  if (loggedIn && allDevices && !currentDevice) {
    return (
      <div>
        {allDevices.map((device) => {
          return (
            <div>
              <button
                onClick={() => {
                  setCurrentDevice(device);
                }}
              >
                {device.name}
              </button>
            </div>
          );
        })}
      </div>
    );
  }

  if (!currentDevice) {
    return null;
  }

  return <div>logged in</div>;
}
const root = document.getElementById("root");
if (!root) {
  throw new Error("No root element");
}

ReactDOM.createRoot(root).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
