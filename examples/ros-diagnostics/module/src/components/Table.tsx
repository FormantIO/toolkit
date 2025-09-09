/* eslint-disable no-cond-assign */
import { ModuleData, App, Authentication, KeyValue } from "@formant/data-sdk";
import { TableComponent } from "./TableComponent/index";
import { useState, FC, useEffect, useRef, useCallback, useMemo } from "react";
import { ModuleConfig } from "./ModuleConfig";
import { ErrorMsg } from "../components/ErrorMsg/ErrorMsg";
import { OnlineTopics } from "../types/RosTopicStats";
import { v4 as uuidv4 } from "uuid";

//Listens to the latest data publish to Formant

type Topic = {
  name: string;
  topic: string;
  minHz: string;
};

export const Table: FC = () => {
  const ros = useRef([]);
  const [latestTopics, setLatestTopics] = useState<OnlineTopics>({
    default: { section: "default", contents: {} },
  });
  const [err, setErr] = useState("Loading Data");
  const [onlineTopics, setOnlineTopics] = useState<string[]>([]);
  const [openConfig, setOpenConfig] = useState(false);
  const [showSnackBar, setShowSnackBar] = useState(false);
  const [currentConfig, setCurrenConfig] = useState();

  useEffect(() => {
    App.addModuleDataListener(receiveModuleData);
  }, []);

  useEffect(() => {
    getCurrentConfig();
  }, [openConfig, showSnackBar]);

  const getCurrentConfig = useCallback(async () => {
    if (await Authentication.waitTilAuthenticated()) {
      try {
        let config = await KeyValue.get("rosDiagnosticsConfiguration");
        setCurrenConfig(JSON.parse(config));
      } catch (e) {
        throw new Error(e as string);
      }
    }
  }, []);

  const receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }
      const items: { name: string; type: string; hz: number }[] = await (
        await fetch(url)
      ).json();
      //Get online topics and group them under "default" section
      //Using uuid as key to be able to reference path ex. state[id] = {section: ""}
      setOnlineTopics(items.map((_) => _.name));
      setLatestTopics({
        [uuidv4()]: {
          section: "default",
          contents: items.reduce(
            (_, topic) => ({
              ..._,
              [uuidv4()]: {
                //Had to use "topicName", because "name" had an issue when updating state
                topicName: topic.name,
                type: topic.type,
                hz: topic.hz,
                enabled: true,
              },
            }),
            {}
          ),
        },
      });
      setErr("");
    } catch (error) {
      ros.current = [];
      setErr(`Error: ${error}`);
    }
  };

  return err.length > 1 ? (
    <ErrorMsg msg={"Error: No data"} />
  ) : openConfig ? (
    <ModuleConfig
      closeConfig={() => setOpenConfig(false)}
      topicStats={latestTopics}
      showSnackBar={() => setShowSnackBar(true)}
      currentConfiuration={currentConfig}
    />
  ) : (
    <TableComponent
      currentConfiuration={currentConfig}
      topicStats={latestTopics}
      tableHeaders={["Name", "Type", "Hz"]}
      setOpenConfig={() => setOpenConfig(true)}
      openSnackBar={() => setShowSnackBar(true)}
      onlineTopics={onlineTopics}
    />
  );
};
function getLatestJsonUrl(moduleData: ModuleData): string | undefined {
  const streams = Object.values(moduleData.streams);
  if (streams.length === 0) {
    throw new Error("No streams.");
  }
  const stream = streams[0];
  if (stream === undefined) {
    throw new Error("No stream.");
  }
  if (stream.loading) {
    return undefined;
  }
  if (stream.tooMuchData) {
    throw new Error("Too much data.");
  }
  if (stream.data.length === 0) {
    throw new Error("No data.");
  }
  const latestPoint = stream.data[0].points.at(-1);
  if (!latestPoint) {
    throw new Error("No datapoints.");
  }
  return latestPoint[1];
}
export default Table;
