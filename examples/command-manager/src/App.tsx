import { useEffect, useState, useMemo, useCallback } from "react";
import "./App.css";
import { useDevice, LoadingIndicator } from "@formant/ui-sdk";
import { Authentication, KeyValue, Device } from "@formant/data-sdk";
import { IConfiguration } from "./types";
import { useSelector, useDispatch } from "react-redux";
import { Commands } from "./components/Commands";
import { setCommands } from "./features/configuration/configurationSlice";
import { useConfiguration } from "./hooks/useConfiguration";
import { useStreams } from "./hooks/useStreams";

interface IStream {
  streamName: string;
  active: boolean;
}

const setStreamAsActive = async (stream: any) => {
  if (await Authentication.waitTilAuthenticated()) {
    const response = await fetch(
      `https://api.formant.io/v1/admin/streams/${stream.id}`,
      {
        method: "PATCH",
        body: JSON.stringify({ ...stream, active: true }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
  }
};

function App() {
  const dispatch = useDispatch();
  const [configuration, loading] = useConfiguration();
  const [paramsFromStreams, setParamsFromStreams] = useState<IStream[]>([]);
  const device = useDevice();
  const streams: IStream[] = useStreams(device);

  useEffect(() => {
    if (!device || !configuration) return;
    const paramStream = configuration.commands.filter(
      (_) => _.streamName.length > 0
    );
    const streamsInfo = streams.filter(
      (_) =>
        paramStream.map((p) => p.streamName).includes(_.streamName) && !_.active
    );

    setParamsFromStreams(streamsInfo);
    dispatch(setCommands({ items: configuration }));
  }, [device, configuration]);

  useEffect(() => {
    if (paramsFromStreams.length === 0) return;
    paramsFromStreams.map((_) => setStreamAsActive(_));
  }, [paramsFromStreams]);

  return (
    <div className="App">
      {loading || !configuration ? (
        <div
          style={{
            minHeight: "100vh",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          <LoadingIndicator />
        </div>
      ) : (
        <Commands device={device as any} configuration={configuration} />
      )}
    </div>
  );
}

export default App;
