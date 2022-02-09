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
    const { device } = this.state;
    if (!device) return;
    device.sendCommand(this.props.command, "");
  };

  render(): ReactNode {
    const { command } = this.props;
    const { device } = this.state;

    return (
      <div className="container">
        <span>{device?.name || "Device.name"}</span>
        <div className="command-name">
          <CommandIcon />
          <span>{command}</span>
        </div>
        <button disabled={!device} onClick={this.issueCommand}>
          Send
        </button>
      </div>
    );
  }
}
