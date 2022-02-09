import { Component, ReactNode } from "react";
import {
  ModuleData,
  App,
  Device,
  Authentication,
  Fleet,
} from "@formant/data-sdk";
import { Docker } from "./DockerImage/index";

interface IDockerImage {
  latestImage: string;
  device: Device | undefined;
  image: string;
  date: string;
}

interface IDockerImageProps {
  command: string;
}

export class DockerImage extends Component<IDockerImageProps, IDockerImage> {
  public constructor(props: any) {
    super(props);
    this.state = {
      device: undefined,
      latestImage: "",
      image: "first image",
      date: "8:30:24 am",
    };
  }

  public componentDidMount = async () => {
    App.addModuleDataListener(this.receiveModuleData);
    if (await Authentication.waitTilAuthenticated()) {
      this.setState({ device: await Fleet.getCurrentDevice() });
    }
  };
  render(): ReactNode {
    return (
      <Docker
        date={this.state.date}
        getLatestImage={this.sendCommand}
        latestImage={this.state.latestImage}
        image={this.state.image}
      />
    );
  }

  private sendCommand = async () => {
    // const { device } = this.state;
    // if (!device) {

    const today = new Date();
    const time =
      today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
    this.setState({ image: "Another image", date: `${time} am` });
    return;
    // }
    // device?.sendCommand(this.props.command, "");
  };

  private receiveModuleData = async (newValue: ModuleData) => {
    try {
      const dockerImage = getLatestJsonImage(newValue);
      if (!dockerImage) {
        return;
      }
      this.setState({ latestImage: dockerImage });
    } catch (error) {
      this.setState({ latestImage: "" });
    }
  };
}

function getLatestJsonImage(moduleData: ModuleData): string | undefined {
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
