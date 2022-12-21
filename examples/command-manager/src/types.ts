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
  streamName: string;
  needsConfirmation: boolean;
  parameterValue: string | null;
}

export interface IConfiguration {
  commands: ICommandConfiguration[];
}
