type AppMessage =
  | { type: "go_to_time"; time: number }
  | { type: "request_module_data"; module: string }
  | { type: "show_message"; message: string }
  | { type: "refresh_auth_token"; module: string }
  | {
      type: "setup_module_menus";
      module: string;
      menus: { label: string }[];
    };

type EmbeddedAppMessage =
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

  private static getCurrentModuleContext(): string | null {
    let urlParams = new URLSearchParams("");

    if (typeof window !== "undefined") {
      urlParams = new URLSearchParams(window.location.search);
    }

    const module = urlParams.get("module");

    return module;
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

  static showMessage(message: string) {
    this.sendAppMessage({ type: "show_message", message });
  }

  static requestModuleData() {
    const module = this.getCurrentModuleContext();
    if (!module) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "request_module_data",
      module,
    });
  }

  static refreshAuthToken() {
    const module = this.getCurrentModuleContext();
    if (!module) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "refresh_auth_token",
      module,
    });
  }

  static setupModuleMenus(menus: { label: string }[]) {
    const module = this.getCurrentModuleContext();
    if (!module) {
      throw new Error("No module context");
    }
    this.sendAppMessage({
      type: "setup_module_menus",
      module,
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
}
