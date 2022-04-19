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
      let device = await Fleet.getDevice(
        "ac8016df-102e-4e97-b194-2bd6bbfa0e4a"
      );
      device.sendCommand(this.props.command, "");
    }
  };

  render(): ReactNode {
    return (
      <div className="container">
        <button onClick={this.issueCommand}>{this.props.command}</button>
      </div>
    );
  }
}
