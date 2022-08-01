/* eslint-disable no-cond-assign */
import { ModuleData, App, Authentication, KeyValue } from "@formant/data-sdk";
import { TableComponent } from "./TableComponent/index";
import { useState, FC, useEffect, useRef, useCallback } from "react";
import { ModuleConfig } from "./ModuleConfig";
import {
  createJsonSchemaObjectFromConfig,
  splitTopicsForSecction,
} from "./TableComponent/utils/index";
import { ErrorMsg } from "../components/ErrorMsg/ErrorMsg";
import { RosTopicStats } from "../types/RosTopicStats";
//Listens to the latest data publish to Formant

type Topic = {
  name: string;
  topic: string;
  minHz: string;
};

export const Table: FC = () => {
  const ros = useRef([]);
  const [latestTopics, setLatestTopics] = useState<{
    [key: string]: RosTopicStats[];
  }>({ default: [] });

  const [err, setErr] = useState("Loading Data");
  const [onlineTopics, setOnlineTopics] = useState<string[]>([]);
  const [openConfig, setOpenConfig] = useState(false);
  const [showSnackBar, setShowSnackBar] = useState(false);
  const [currentConfig, setCurrenConfig] = useState();
  const [jsonObjectFromCloud, setJsonObjectFromCloud] = useState<any>();
  const [cloudConfig, setCloudConfig] = useState<{
    [key: string]: {
      title: string;
      contents: Topic[];
    };
  }>({
    other: {
      title: "Other",
      contents: [],
    },
  });
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
        setJsonObjectFromCloud(
          createJsonSchemaObjectFromConfig(JSON.parse(config))
        );
        setCloudConfig(splitTopicsForSecction(JSON.parse(config)));
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
      const items = await (await fetch(url)).json();
      // setOnlineTopics(items.map((_: { name: string }) => _.name));
      //Get online topics and group them under "default" section
      setLatestTopics({ default: items });
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
      topicStats={latestTopics.items}
      showSnackBar={() => setShowSnackBar(true)}
      jsonObjectFromCloud={jsonObjectFromCloud}
      currentConfiuration={currentConfig}
    />
  ) : (
    <TableComponent
      currentConfiuration={currentConfig}
      topicStats={latestTopics}
      tableHeaders={["Name", "Type", "Hz"]}
      setOpenConfig={() => setOpenConfig(true)}
      cloudConfig={cloudConfig}
      setShowSnackBar={() => setShowSnackBar(false)}
      openSnackBar={() => setShowSnackBar(true)}
      showSnackBar={showSnackBar}
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
