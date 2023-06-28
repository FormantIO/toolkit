import { Authentication } from "./Authentication";
import { FORMANT_API_URL } from "./config";
import { QueryStore } from "./cache/queryStore";
import { StreamType } from "./model/StreamType";
import { IStreamData } from "./model/IStreamData";
import { JsonSchema } from "./model/JsonSchema";
import { sendAppMessage } from "./message-bus/sendAppMessage";
import {
  EmbeddedAppMessage,
  IDevice,
  ModuleConfigurationMessage,
} from "./message-bus/EmbeddedAppMessage";
import { getCurrentModuleContext } from "./utils/getCurrentModuleContext";

const queryStore = new QueryStore();

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
  static getCurrentModuleContext(): string | null {
    return getCurrentModuleContext();
  }

  static async getCurrentModuleConfiguration(): Promise<string | undefined> {
    let urlParams = new URLSearchParams("");

    if (typeof window !== "undefined" && window.location) {
      urlParams = new URLSearchParams(window.location.search);
    }

    const configurationId = urlParams.get("configuration");

    if (configurationId === null || configurationId.trim() === "") {
      return undefined;
    }

    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/module-configurations/` + configurationId,
      {
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const moduleConfiguration = await response.json();
    return moduleConfiguration.configuration;
  }

  static isModule(): boolean {
    return getCurrentModuleContext() !== null;
  }

  static goToTime(date: Date) {
    sendAppMessage({
      type: "go_to_time",
      time: date.getTime(),
    });
  }

  static goToDevice(deviceId: string) {
    sendAppMessage({
      type: "go_to_device",
      deviceId,
    });
  }

  static showMessage(message: string) {
    sendAppMessage({ type: "show_message", message });
  }

  static requestModuleData() {
    const moduleName = getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    sendAppMessage({
      type: "request_module_data",
      module: moduleName,
    });
  }

  static setModuleDateTimeRange(
    beforeInMilliseconds: number,
    afterInMilliseconds?: number
  ) {
    const moduleName = getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    sendAppMessage({
      type: "set_module_data_time_range",
      module: moduleName,
      before: beforeInMilliseconds,
      after: afterInMilliseconds || 0,
    });
  }

  static refreshAuthToken() {
    Authentication.refreshAuthToken();
  }

  static sendChannelData(channel: string, data: any) {
    const moduleName = getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    sendAppMessage({
      type: "send_channel_data",
      source: moduleName,
      channel,
      data,
    });
  }

  static setupModuleMenus(menus: { label: string }[]) {
    const moduleName = getCurrentModuleContext();
    if (!moduleName) {
      throw new Error("No module context");
    }
    sendAppMessage({
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
    return Authentication.addAccessTokenRefreshListener(handler);
  }

  static addModuleDataListener(handler: (data: ModuleData) => void) {
    const moduleName = getCurrentModuleContext();
    if (moduleName) {
      sendAppMessage({ type: "request_module_data", module: moduleName });
    }
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

  static addOverviewDeviceListener(handler: (devices: IDevice[]) => void) {
    sendAppMessage({ type: "request_devices" });
    const listener = (event: MessageEvent<any>) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "overview_devices") {
        handler(msg.data);
      }
    };
    window.addEventListener("message", listener);

    return () => window.removeEventListener("message", listener);
  }

  static addStreamListener(
    streamNames: string[],
    streamTypes: StreamType[],
    handler: (response: IStreamData[] | "too much data" | undefined) => void
  ): () => void {
    const listener = (event: any) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "module_data") {
        const { start, end } = msg.queryRange;
        handler(
          queryStore.moduleQuery(
            {},
            streamNames,
            streamTypes,
            new Date(start),
            new Date(end),
            false
          )
        );
      }
    };
    window.addEventListener("message", listener);
    return () => window.removeEventListener("message", listener);
  }

  static addModuleConfigurationListener(
    handler: (event: ModuleConfigurationMessage) => void
  ) {
    window.addEventListener("message", (event) => {
      const msg = event.data as EmbeddedAppMessage;
      if (msg.type === "module_configuration") {
        handler(msg as ModuleConfigurationMessage);
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

  static async prompt(
    schema: JsonSchema,
    options?: { okText?: string; cancelText?: string }
  ): Promise<any> {
    return new Promise((resolve) => {
      const promptId = Math.random().toString();
      sendAppMessage({
        type: "prompt",
        promptId,
        schema,
        okText: options?.okText,
        cancelText: options?.cancelText,
      });
      const handler = (event: any) => {
        const msg = event.data as EmbeddedAppMessage;
        if (msg.type === "prompt_response" && msg.promptId === promptId) {
          resolve(msg.data);
        }
        window.removeEventListener("message", handler);
      };
      window.addEventListener("message", handler);
    });
  }

  static async getDate(time?: Date, minTime?: Date, maxTime?: Date) {
    return new Promise((resolve) => {
      sendAppMessage({
        type: "request_date",
        minTime,
        maxTime,
        time,
      });
      const handler = (event: any) => {
        const msg = event.data as EmbeddedAppMessage;
        if (msg.type === "date_response") {
          window.removeEventListener("message", handler);
          resolve(msg.data);
        }
      };
      window.addEventListener("message", handler);
    });
  }

  static async disableAnalyticsBottomBar() {
    sendAppMessage({
      type: "hide_analytics_date_picker",
    });
  }
  private static _isOnline: boolean | null = null;

  private static _handleOnlineEvent = (e: MessageEvent<EmbeddedAppMessage>) => {
    const { data } = e;
    if (data.type === "formant_online") {
      this._isOnline = data.online;
    }
  };

  static get isOnline(): boolean | null {
    return App._isOnline;
  }

  static listenForConnectionEvents() {
    window.addEventListener("message", this._handleOnlineEvent);
  }

  static checkConnection(deadlineMs: number = 1_000): Promise<boolean> {
    return new Promise((done, reject) => {
      const deadline = setTimeout(
        () => reject(new Error("deadline expired: took too long")),
        deadlineMs
      );

      const handler = (e: MessageEvent<EmbeddedAppMessage>) => {
        window.removeEventListener("message", handler);
        clearTimeout(deadline);

        const { data } = e;
        if (data.type === "formant_online") {
          this._isOnline = data.online;
          done(data.online);
        }
      };

      window.addEventListener("message", handler);
      sendAppMessage({ type: "formant_online" });
    });
  }

  static waitForConnection(deadlineMs: number = 5_000): Promise<void> {
    let aborted = false;
    const deadline = new Promise<void>((_, reject) => {
      setTimeout(() => {
        aborted = true;
        reject(new Error("deadline expired: took too long"));
      }, deadlineMs);
    });

    const delay = (ms: number) => new Promise((done) => setTimeout(done, ms));

    const loop = async (): Promise<void> => {
      await delay(50); // allow for initialization jitter to settle
      while (!aborted) {
        if (this.isOnline || (await this.checkConnection)) {
          break;
        }
        await delay(500);
      }
    };

    return Promise.race([deadline, loop()]);
  }
}
