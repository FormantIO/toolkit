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
  errorMessage: string;
}

export class DockerImage extends Component<{}, IDockerImage> {
  public constructor(props: any) {
    super(props);
    this.state = {
      latestImage: "",
      errorMessage: "Waiting for data...",
    };
  }

  public componentDidMount() {
    App.addModuleDataListener(this.receiveModuleData);
  }
  render(): ReactNode {
    return (
      <Docker
        getLatestImage={this.sendCommand}
        latestImage={this.state.latestImage}
      />
    );
  }

  private sendCommand = async () => {
    console.log("test");
    if (await Authentication.waitTilAuthenticated()) {
      let currentDevice = await Fleet.getCurrentDevice();
      let cDevice = new Device(
        currentDevice.id,
        currentDevice.name,
        currentDevice.organizationId
      );
      console.log(cDevice);
      await cDevice.sendCommand("Fake update docker", "mydata");
      // console.log(await cDevice.getAvailableCommands());
    }
  };
  private receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }

      this.setState({ latestImage: url });
    } catch (error) {
      this.setState({ latestImage: "", errorMessage: error as string });
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
