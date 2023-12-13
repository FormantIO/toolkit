export interface ICommand {
  name: string;
  id: string;
  command: string;
  description: string;
  parameterEnabled: string;
  parameterValue: string;
  parameterMeta: string;
  enabled: boolean;
}

export interface activeCommands {
  activeCommands: ICommand[];
}

export interface ICommandConfiguration {
  name: string;
  enabledParameters: boolean;
  useStreamValue: boolean;
  streamName: string;
}

export interface IButtonConfiguration {
  streamName: string;
}

export interface IConfiguration {
  commands: ICommandConfiguration[];
  buttons: IButtonConfiguration[];
}
