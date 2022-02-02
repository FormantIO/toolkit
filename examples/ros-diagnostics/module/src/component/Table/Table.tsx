/* eslint-disable no-cond-assign */
import { Component } from "react";
import { ModuleData, App } from "@formant/data-sdk";

import RosTopicStats from "../../types/RosTopicStats";
import { TableComponent } from "../TableComponent/index";
import { ErrorMsg } from "../ErrorMsg/ErrorMsg";
interface ITableState {
  latestStats?: RosTopicStats[];
  errorMessage?: string;
}
class Table extends Component<{}, ITableState> {
  public constructor(props: any) {
    super(props);
    this.state = {
      latestStats: undefined,
      errorMessage: "Waiting for data...",
    };
  }
  public componentDidMount() {
    App.addModuleDataListener(this.receiveModuleData);
  }

  public render() {
    const { errorMessage, latestStats } = this.state;
    const tableContentsVisible = !!latestStats;
    const message = errorMessage ?? "Loading...";
    return tableContentsVisible ? (
      <>
        <TableComponent
          topicStats={latestStats}
          tableHeaders={["Section", "Name", "Type", "Hz"]}
        />
      </>
    ) : (
      <ErrorMsg msg={message.toString()} />
    );
  }

  private receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }
      const latestStats = await (await fetch(url)).json();
      this.setState({ latestStats });
    } catch (error) {
      this.setState({ latestStats: undefined, errorMessage: error as string });
    }
  };
}
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
