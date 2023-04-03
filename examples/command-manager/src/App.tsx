import "./App.css";
import { useDevice, LoadingIndicator, useFormant } from "@formant/ui-sdk";
import { IConfiguration } from "./types";
import { CommandRow } from "./CommandRow";
import { useStreams } from "./hooks/useStreams";
import { useCommands } from "./hooks/useCommands";
import styled from "@emotion/styled";
import { useEffect } from "react";

export interface IStream {
  streamName: string;
  active: boolean;
}

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const device = useDevice();
  const streams: IStream[] = useStreams(device);
  const commands = useCommands();

  return (
    <div className="App">
      {!config ? (
        <Conatiner>
          <LoadingIndicator />
        </Conatiner>
      ) : (
        <Table>
          {config.commands.map((_) => {
            return (
              <CommandRow
                device={device}
                name={_.name}
                description={
                  commands?.filter((command) => _.name === command.name)[0]
                    ?.description ?? ""
                }
                streamName={_.streamName ?? ""}
                streams={!!_.streamName ? streams : []}
                useStreamValue={_.useStreamValue}
                enabledParameters={_.enabledParameters}
                parameterValue={
                  commands?.filter((command) => _.name === command.name)[0]
                    ?.parameterValue ?? ""
                }
                meta={
                  commands?.filter((command) => _.name === command.name)[0]
                    ?.parameterMeta ?? {}
                }
              />
            );
          })}
        </Table>
      )}
    </div>
  );
}

export default App;

const Conatiner = styled.div`
  min-height: 100vh;
  display: flex;
  align-items: center;
  flex-direcction: column;
`;

const Table = styled.table`
  width: 100%;
  border-collapse: collapse;
`;
