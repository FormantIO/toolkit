import { getModuleConfiguration } from "./api/getModuleConfiguration";
import { getCurrentModuleContext } from "./utils/getCurrentModuleContext";
import { disableAnalyticsBottomBar } from "./message-bus/senders/disableAnalyticsBottomBar";
import { goToDevice } from "./message-bus/senders/goToDevice";
import { goToTime } from "./message-bus/senders/goToTime";
import { refreshAuthToken } from "./message-bus/senders/refreshAuthToken";
import { requestModuleData } from "./message-bus/senders/requestModuleData";
import { sendAppMessage } from "./message-bus/senders/sendAppMessage";
import { sendChannelData } from "./message-bus/senders/sendChannelData";
import { setModuleDateTimeRange } from "./message-bus/senders/setModuleDateTimeRange";
import { setupModuleMenus } from "./message-bus/senders/setupModuleMenus";
import { showMessage } from "./message-bus/senders/showMessage";
import { EmbeddedAppMessage } from "./message-bus/listeners/EmbeddedAppMessage";
import { addAccessTokenRefreshListener } from "./message-bus/listeners/addAccessTokenRefreshListener";
import { addChannelDataListener } from "./message-bus/listeners/addChannelDataListener";
import { addMenuListener } from "./message-bus/listeners/addMenuListener";
import { addModuleConfigurationListener } from "./message-bus/listeners/addModuleConfigurationListener";
import { addModuleDataListener } from "./message-bus/listeners/addModuleDataListener";
import { addOverviewDeviceListener } from "./message-bus/listeners/addOverviewDeviceListener";
import { addStreamListener } from "./message-bus/listeners/addStreamLIstener";
import { getDate } from "./message-bus/bidirectional/getDate";
import { prompt } from "./message-bus/bidirectional/prompt";

export class App {
  static getCurrentModuleContext = getCurrentModuleContext;

  static isModule(): boolean {
    return getCurrentModuleContext() !== null;
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

    return getModuleConfiguration(configurationId.trim());
  }

  // senders
  static disableAnalyticsBottomBar = disableAnalyticsBottomBar;
  static goToDevice = goToDevice;
  static goToTime = goToTime;
  static refreshAuthToken = refreshAuthToken;
  static requestModuleData = requestModuleData;
  static sendChannelData = sendChannelData;
  static setModuleDateTimeRange = setModuleDateTimeRange;
  static setupModuleMenus = setupModuleMenus;
  static showMessage = showMessage;

  // listeners
  static addAccessTokenRefreshListener = addAccessTokenRefreshListener;
  static addChannelDataListener = addChannelDataListener;
  static addMenuListener = addMenuListener;
  static addModuleConfigurationListener = addModuleConfigurationListener;
  static addModuleDataListener = addModuleDataListener;
  static addOverviewDeviceListener = addOverviewDeviceListener;
  static addStreamListener = addStreamListener;

  // bidirectional
  static getDate = getDate;
  static prompt = prompt;

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
