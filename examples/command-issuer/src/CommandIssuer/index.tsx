import { Component, ReactNode } from "react";
import { CommandIcon } from "./CommandIcon";
import { Authentication, Fleet, Device } from "@formant/data-sdk";

interface ICommandIssuerProps {
  command: string;
}
interface ICommandIssuerState {
  device: Device | undefined;
}

export class CommandIssuer extends Component<
  ICommandIssuerProps,
  ICommandIssuerState
> {
  public constructor(props: any) {
    super(props);
    this.state = {
      device: undefined,
    };
  }

  public componentDidMount = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      this.setState({ device: await Fleet.getCurrentDevice() });
    }
  };

  private issueCommand = async () => {
    if (Authentication.isAuthenticated()) {
      let x = await Fleet.getDevice("ac8016df-102e-4e97-b194-2bd6bbfa0e4a");
      x.sendCommand("fake.update.docker", "reset all");
    }
    // const { device } = this.state;
    // if (!device) return;
  };

  render(): ReactNode {
    const { command } = this.props;
    const { device } = this.state;

    return (
      <div className="container">
        <span>{device?.name || "Device.name"}</span>
        <div className="command-name">
          <CommandIcon />
          <span>{"fake.update.docker"}</span>
        </div>
        <button onClick={this.issueCommand}>Send</button>
      </div>
    );
  }
}
