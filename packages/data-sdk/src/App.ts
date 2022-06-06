export type AppMessage =
  | { type: "go_to_time"; time: number }
  | { type: "go_to_device"; deviceId: string }
  | { type: "request_module_data"; module: string }
  | { type: "show_message"; message: string }
  | { type: "refresh_auth_token"; module: string }
  | {
      type: "set_module_data_time_range";
      module: string;
      before: number;
      after: number;
    }
  | {
      type: "setup_module_menus";
      module: string;
      menus: { label: string }[];
    }
  | {
      type: "send_channel_data";
      channel: string;
      source: string;
      data: any;
    };

export type EmbeddedAppMessage =
  | {
      type: "module_menu_item_clicked";
      menu: string;
    }
  | {
      type: "auth_token";
      token: string;
    }
  | {
      type: "module_data";
      streams: { [x: string]: any };
      time: number;
      queryRange: { start: number; end: number };
    }
  | {
      type: "channel_data";
      channel: string;
      source: string;
      data: any;
    };
export interface ModuleData {
  queryRange: QueryRange;
  time: number;
  streams: { [stream_name: string]: Stream };
}
export interface QueryRange {
  start: number;
  end: number;
}
export interface Stream {
  data: StreamData[];
  loading: boolean;
  tooMuchData: boolean;
  type: string;
}
export interface StreamData {
  points: DataPoint[];
  deviceId: string;
  agentId: string;
  name: string;
  tags: { [key: string]: string };
  type: string;
}

export type DataPoint = [number, any];

export class App {
  private static sendAppMessage(message: AppMessage) {
    window.parent.postMessage(message, "*");
  }

  static getCurrentModuleContext(): string | null {
    let urlParams = new URLSearchParams("");

    if (typeof window !== "undefined") {
      urlParams = new URLSearchParams(window.location.search);
    }

    const moduleName = urlParams.get("module");

    return moduleName;
  }

  static isModule(): boolean {
    return this.getCurrentModuleContext() !== null;
  }

  static goToTime(date: Date) {
    this.sendAppMessage({
      type: "go_to_time",
      time: date.getTime(),
    });
  }

  static goToDevice(deviceId: string) {
    this.sendAppMessage({
      type: "go_to_device",
      deviceId,
    });
  }

  static showMessage(message: string) {
    this.sendAppMessage({ type: "show_message", message });
  }

  static requestModuleData() {
    const moduleName = this.getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "request_module_data",
      module: moduleName,
    });
  }

  static setModuleDateTimeRange(
    beforeInMilliseconds: number,
    afterInMilliseconds?: number
  ) {
    const moduleName = this.getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "set_module_data_time_range",
      module: moduleName,
      before: beforeInMilliseconds,
      after: afterInMilliseconds || 0,
    });
  }

  static refreshAuthToken() {
    const moduleName = this.getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "refresh_auth_token",
      module: moduleName,
    });
  }

  static sendChannelData(channel: string, data: any) {
    const moduleName = this.getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "send_channel_data",
      source: moduleName,
      channel,
      data,
    });
  }

  static setupModuleMenus(menus: { label: string }[]) {
    const moduleName = this.getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "setup_module_menus",
      module: moduleName,
      menus,
    });
  }

  static addMenuListener(handler: (label: string) => void) {
    window.addEventListener("message", (event) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "module_menu_item_clicked") {
        handler(msg.menu);
      }
    });
  }

  static addAccessTokenRefreshListener(handler: (token: string) => void) {
    window.addEventListener("message", (event) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "auth_token") {
        handler(msg.token);
      }
    });
  }

  static addModuleDataListener(handler: (data: ModuleData) => void) {
    window.addEventListener("message", (event) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "module_data") {
        handler({
          streams: msg.streams,
          time: msg.time,
          queryRange: msg.queryRange,
        });
      }
    });
  }

  static addChannelDataListener(
    channel: string,
    handler: (e: { source: string; data: any }) => void
  ) {
    window.addEventListener("message", (event) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "channel_data" && msg.channel === channel) {
        handler({
          source: msg.source,
          data: msg.data,
        });
      }
    });
  }
}
(";");
