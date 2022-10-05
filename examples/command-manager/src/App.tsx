import { useEffect, useState, useMemo, useCallback } from "react";
import "./App.css";
import { useDevice } from "@formant/ui-sdk";
import { Authentication, KeyValue } from "@formant/data-sdk";
import { Configuration } from "./components/Configuration";
import { ICommand } from "./types";
import { useSelector, useDispatch } from "react-redux";
import { Commands } from "./components/Commands";
import { updateActiveCommands } from "./features/configuration/configurationSlice";

const getCommands = async (): Promise<ICommand[]> => {
  if (await Authentication.waitTilAuthenticated()) {
    const result = await fetch(
      `https://api.formant.io/v1/admin/command-templates/`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const commandsList = await result.json();
    return commandsList.items.filter((_: ICommand) => _.enabled);
  }
  return [];
};

const getCloudConfiguration = async (moduleName: string) => {
  if (await Authentication.waitTilAuthenticated()) {
    const result = await fetch(
      `https://api.formant.io/v1/admin/key-value/${moduleName}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const parseResult = await result.json();
    if (parseResult.message) return [];
    return JSON.parse(parseResult.value);
  }
};

function App() {
  const device = useDevice();
  const moduleName = useSelector((state: any) => state.moduleName.moduleName);
  const activeCommands = useSelector(
    (state: any) => state.configuration.activeCommands
  );

  const dispatch = useDispatch();
  const [commands, setCommands] = useState<ICommand[]>([]);
  const [showConfig, setShowConfig] = useState(false);

  useEffect(() => {
    if (!device) return;
    getCommands().then((_) => setCommands(_));
    getCloudConfiguration(moduleName).then((_) =>
      dispatch(updateActiveCommands({ items: _ }))
    );
  }, [device]);

  const displayCommands = useMemo(() => {
    return commands.filter((_) => activeCommands.includes(_.id));
  }, [activeCommands, commands]);

  const handleOpenconfiguration = useCallback(() => {
    setShowConfig(true);
  }, []);

  const handleCloseconfiguration = useCallback(async () => {
    setShowConfig(false);

    if (await Authentication.waitTilAuthenticated()) {
      await KeyValue.set(moduleName, JSON.stringify(activeCommands));
    }
  }, [activeCommands]);

  const handleIssueCommand = useCallback(
    (name: string, value: string | null) => {
      device.sendCommand(name, value === null ? "" : value);
    },
    [device]
  );

  return (
    <div className="App">
      {showConfig ? (
        <Configuration
          handleCloseconfiguration={handleCloseconfiguration}
          commands={commands}
        />
      ) : (
        <Commands
          handleIssueCommand={handleIssueCommand}
          commands={displayCommands}
          openConfiguration={handleOpenconfiguration}
        />
      )}
    </div>
  );
}

export default App;
