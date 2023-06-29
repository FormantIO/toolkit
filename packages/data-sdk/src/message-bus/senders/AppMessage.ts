import { JsonSchema } from "../../model/JsonSchema";

export interface RequestDateMessage {
  type: "request_date";
  minTime?: Date;
  maxTime?: Date;
  time?: Date;
}

export interface GoToTimeMessage {
  type: "go_to_time";
  time: number;
}

export interface PromptMessage {
  type: "prompt";
  promptId: string;
  schema: JsonSchema;
  okText?: string;
  cancelText?: string;
}

export interface GoToDeviceMessage {
  type: "go_to_device";
  deviceId: string;
}

export interface RequestModuleDataMessage {
  type: "request_module_data";
  module: string;
}

export interface ShowMessageMessage {
  type: "show_message";
  message: string;
}

export interface RefreshAuthTokenMessage {
  type: "refresh_auth_token";
  module: string;
}

export interface SetModuleDataTimeRangeMessage {
  type: "set_module_data_time_range";
  module: string;
  before: number;
  after: number;
}

export interface SetupModuleMenusMessage {
  type: "setup_module_menus";
  module: string;
  menus: { label: string }[];
}

export interface SendChannelDataMessage {
  type: "send_channel_data";
  channel: string;
  source: string;
  data: any;
}

export interface RequestDevicesMessage {
  type: "request_devices";
}

export interface HideAnalyticsDatePickerMessage {
  type: "hide_analytics_date_picker";
}

export interface FormantOnlineMessage {
  type: "formant_online";
}

export type AppMessage =
  | RequestDateMessage
  | GoToTimeMessage
  | PromptMessage
  | GoToDeviceMessage
  | RequestModuleDataMessage
  | ShowMessageMessage
  | RefreshAuthTokenMessage
  | SetModuleDataTimeRangeMessage
  | SetupModuleMenusMessage
  | SendChannelDataMessage
  | RequestDevicesMessage
  | HideAnalyticsDatePickerMessage
  | FormantOnlineMessage;
