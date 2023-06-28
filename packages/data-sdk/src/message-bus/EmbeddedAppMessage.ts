import { ITags } from "../model/ITags";

export type DateResponseMessage = {
  type: "date_response";
  data: Date;
};

export interface IDevice {
  name: string;
  id: string;
  tags: ITags;
}

export type OverviewDevicesMessage = {
  type: "overview_devices";
  data: IDevice[];
};

export type ModuleMenuItemClickedMessage = {
  type: "module_menu_item_clicked";
  menu: string;
};

export type ModuleConfigurationMessage = {
  type: "module_configuration";
  temporary: boolean;
  configuration: string;
};

export type AuthTokenMessage = {
  type: "auth_token";
  token: string;
};

export type ModuleDataMessage = {
  type: "module_data";
  streams: { [x: string]: any };
  time: number;
  queryRange: { start: number; end: number };
};

export type ChannelDataMessage = {
  type: "channel_data";
  channel: string;
  source: string;
  data: any;
};

export type PromptResponseMessage = {
  type: "prompt_response";
  promptId: string;
  data: any;
};

export type FormantOnlineMessage = {
  type: "formant_online";
  online: boolean;
};

export type EmbeddedAppMessage =
  | DateResponseMessage
  | OverviewDevicesMessage
  | ModuleMenuItemClickedMessage
  | AuthTokenMessage
  | ModuleDataMessage
  | ChannelDataMessage
  | PromptResponseMessage
  | FormantOnlineMessage
  | ModuleConfigurationMessage;
