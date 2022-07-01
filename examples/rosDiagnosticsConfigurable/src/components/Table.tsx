/* eslint-disable no-cond-assign */
import { ModuleData, App } from "@formant/data-sdk";
import { TableComponent } from "./TableComponent/index";
import { useState, FC, useEffect, useRef } from "react";

export const Table: FC = () => {
  const ros = useRef([]);
  const [latestTopics, setLatestTopics] = useState({ items: [] });

  useEffect(() => {
    App.addModuleDataListener(receiveModuleData);
  }, []);

  const receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }
      const items = await (await fetch(url)).json();
      if (JSON.stringify(items) === JSON.stringify(ros.current)) return;
      ros.current = items;
      setLatestTopics({ items });
    } catch (error) {
      ros.current = [];
    }
  };

  return (
    <TableComponent
      topicStats={latestTopics.items}
      tableHeaders={["Section", "Name", "Type", "Hz"]}
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
