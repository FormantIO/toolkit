import { Component, ReactNode } from "react";
import moduleConfig from "../../config/moduleConfig";
import { ModuleData, App } from "@formant/data-sdk";
import { BoardContent } from "../BoardContent";
import RosTopicStats from "../../types/RosTopicStats";

interface IBoardState {
  onlineTopics: string[];
  errorMessage: string;
}

export class Board extends Component<{}, IBoardState> {
  public constructor(props: any) {
    super(props);
    this.state = {
      onlineTopics: [],
      errorMessage: "Waiting for data...",
    };
  }

  public componentDidMount() {
    App.addModuleDataListener(this.receiveModuleData);
  }

  render(): ReactNode {
    return <BoardContent onlineTopics={this.state.onlineTopics} />;
  }

  private receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }
      let latestStats = await (await fetch(url)).json();
      latestStats = latestStats.map((_: RosTopicStats) => _.name);

      console.log(latestStats);

      this.setState({
        onlineTopics: latestStats,
      });
    } catch (error) {
      this.setState({ onlineTopics: [], errorMessage: error as string });
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
