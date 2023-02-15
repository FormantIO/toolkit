import { useEffect, useState, useMemo, useCallback } from "react";
import "./App.css";
import {
  useDevice,
  LoadingIndicator,
  useFormant,
  TextField,
  Button,
} from "@formant/ui-sdk";
import { Authentication, KeyValue, Device } from "@formant/data-sdk";
import { IConfiguration } from "./types";
import { useSelector, useDispatch } from "react-redux";
import { Commands } from "./components/Commands";
import { setCommands } from "./features/configuration/configurationSlice";
import { useStreams } from "./hooks/useStreams";
import { CommandRow } from "./CommandRow";

import styled from "@emotion/styled";

interface IStream {
  streamName: string;
  active: boolean;
}

function App() {
  const dispatch = useDispatch();
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const [paramsFromStreams, setParamsFromStreams] = useState<IStream[]>([]);
  const device = useDevice();
  const streams: IStream[] = useStreams(device);

  return (
    <div className="App">
      {!config ? (
        <LoadingIndicator />
      ) : (
        <Table>
          {config.commands.map((_) => {
            return (
              <CommandRow
                name={_.name}
                description={
                  " Lorem ipsum dolor, sit amet consectetur adipisicing elit."
                }
                streamName={_.streamName ?? ""}
              />
            );
          })}
        </Table>
      )}
    </div>
  );
}

export default App;

const Table = styled.table`
  width: 100%;
  border-collapse: collapse;
`;

const Desciption = styled.td`
  width: 50%;
  text-align: left;
`;

const Name = styled.span`
  color: white;
`;
