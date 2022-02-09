import { Component, ReactNode } from "react";
import { ModuleData, App } from "@formant/data-sdk";
import { BoardContent } from "../BoardContent";
import { rosNodes } from "../../config/types/node";
import { ErrorMsg } from "../ErrorMsg/ErrorMsg";

interface IBoardState {
  onlineNodes: rosNodes | undefined;
  errorMessage: string;
}

export class Board extends Component<{}, IBoardState> {
  public constructor(props: any) {
    super(props);
    this.state = {
      onlineNodes: undefined,
      errorMessage: "Waiting for data...",
    };
  }

  public componentDidMount() {
    App.addModuleDataListener(this.receiveModuleData);
  }

  render(): ReactNode {
    const { errorMessage, onlineNodes } = this.state;
    const tableContentsVisible = !!onlineNodes;
    const message = errorMessage ?? "Loading...";
    return tableContentsVisible ? (
      <BoardContent onlineNodes={this.state.onlineNodes!} />
    ) : (
      <ErrorMsg msg={message.toString()} />
    );
  }

  private receiveModuleData = async (newValue: ModuleData) => {
    try {
      const nodes = getLatestJson(newValue);
      if (!nodes) return;
      this.setState({ onlineNodes: nodes });
    } catch (error) {
      this.setState({ onlineNodes: undefined, errorMessage: error as string });
    }
  };
}

function getLatestJson(moduleData: ModuleData): rosNodes | undefined {
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
