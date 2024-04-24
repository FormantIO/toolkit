import { RtcClient, IRtcPeer } from "@formant/realtime-sdk";
import { delay } from "../../../common/delay";
import { IStreamCurrentValue } from "../model/IStreamCurrentValue";
import { ConfigurationDocument } from "./device.types";
import { BaseDevice } from "./BaseDevice";
import { TelemetryResult } from "../model/TelemetryResult";

export class PeerDevice extends BaseDevice {
  id!: string;

  private telemetryStreamActive = false;
  private streamTelemetry: { [key: string]: any } = {};

  constructor(public peerUrl: string) {
    super();
  }

  async getLatestTelemetry(): Promise<IStreamCurrentValue[]> {
    if (!this.telemetryStreamActive) {
      this.subscribeToTelemetry();
    }

    const telemetry = this.streamTelemetry as {
      [key in string]: { timestamp: string };
    };
    const entries = Object.entries(telemetry);
    return entries.map(([stream, latestValue]) => {
      const v: IStreamCurrentValue = {
        deviceId: this.id,
        streamName: stream,
        streamType: "json",
        currentValue: latestValue,
        currentValueTime: latestValue.timestamp,
        tags: {},
      };
      return v;
    });
  }

  async getTelemetry(
    streamNameOrStreamNames: string | string[],
    start: Date,
    end: Date,
    tags?: { [key in string]: string[] },
    limit?: number,
    offset?: number,
    latestOnly?: boolean
  ): Promise<TelemetryResult[]> {
    if (Array.isArray(streamNameOrStreamNames)) {
      throw new Error("Multiple stream names not supported");
    }
    if (tags) {
      throw new Error("Tags not supported");
    }

    if (latestOnly && limit === undefined) {
      limit = 1;
    } else if (latestOnly && limit !== undefined) {
      throw new Error("latestOnly and limit cannot be used together");
    }

    let queryUrl = `${
      this.peerUrl
    }/v1/querydatapoints?stream_name=${streamNameOrStreamNames}&start=${start.toISOString()}&end=${end.toISOString()}`;
    if (limit != null && limit > 0) {
      queryUrl += `&limit=${limit}`;
    }
    if (offset != null && offset >= 0) {
      queryUrl += `&offset=${offset}`;
    }

    const result = await fetch(queryUrl);
    const queryResp = await result.json();

    const telemetryDatapoints: TelemetryResult[] = [];
    for (const dp of queryResp.results) {
      const dpTime = parseInt(dp.timestamp);
      const dpType = dp.tags.data_type;
      delete dp.tags.data_type;

      telemetryDatapoints.push({
        deviceId: this.id,
        name: dp.stream,
        points: [[dpTime, this.getPointPayload(dpType, dp)]],
        tags: dp.tags,
        type: dpType,
      });
    }

    return telemetryDatapoints;
  }

  private getPointPayload(type: string, datapoint: any): any {
    switch (type) {
      case "numeric":
        return datapoint.numeric.value;
      case "numeric_set":
        return datapoint.numericSet.numerics;
      case "text":
        return datapoint.text.value;
      case "bitset":
        const keys = [];
        const values = [];
        for (const bit of datapoint.bitset.bits) {
          keys.push(bit.key);
          values.push(bit.value);
        }
        return {
          keys,
          values,
        };
      case "location":
        return datapoint.location;
      case "health":
        return datapoint.health;
      case "battery":
        return datapoint.battery;
      default:
        return {};
    }
  }

  private subscribeToTelemetry() {
    this.telemetryStreamActive = true;

    let previousBytesReceived = 0;

    // we use XHR because chunked encoding is broken in the react native fetch API
    const xhr = new XMLHttpRequest();
    xhr.responseType = "text";

    xhr.addEventListener("error", (_) => {
      this.handleXHRError("error");
    });
    xhr.addEventListener("abort", (_) => {
      this.handleXHRError("abort");
    });
    xhr.addEventListener("timeout", (_) => {
      this.handleXHRError("timeout");
    });
    xhr.addEventListener("readystatechange", (_) => {
      if (xhr.readyState === XMLHttpRequest.DONE) {
        this.handleXHRError("closed");
      }
    });

    // handle new data
    xhr.addEventListener("progress", (event) => {
      const currentBytesReceived = event.loaded;
      const newBytesReceived = currentBytesReceived - previousBytesReceived;
      previousBytesReceived = currentBytesReceived;

      // Process the new data received
      const newData = xhr.responseText.substr(-newBytesReceived);

      const jsonObjects = newData.split("\n");
      jsonObjects.forEach((jsonObject) => {
        if (jsonObject.length > 0) {
          const parsedObject = JSON.parse(jsonObject);
          if (parsedObject.result?.datapoint) {
            const datapoint = parsedObject.result.datapoint;
            const stream = datapoint.stream;
            delete datapoint["stream"];
            this.streamTelemetry[stream] = datapoint;
          }
        }
      });
    });

    xhr.open("POST", `${this.peerUrl}/v1/telemetry`);

    xhr.send();
  }

  private handleXHRError(reason: string) {
    console.warn(`Telemetry stream ended: ${reason}`);
    this.telemetryStreamActive = false;
  }

  async getDeviceId(): Promise<string> {
    let result = await fetch(`${this.peerUrl}/v1/config`);
    const cfg = await result.json();
    return cfg.configuration.id;
  }

  async getConfiguration(): Promise<ConfigurationDocument> {
    let result = await fetch(`${this.peerUrl}/v1/config`);
    const cfg = await result.json();
    return cfg.configuration.document;
  }

  async startRealtimeConnection(sessionType?: number): Promise<void> {
    console.debug(`${new Date().toISOString()} :: Connection start requested`);

    if (this.rtcClient && this.connectionMonitorInterval !== undefined) {
      throw new Error(
        `Already created realtime connection to device ${this.id}`
      );
    }

    if (this.rtcClient) {
      console.warn(
        "overwriting existing rtcClient due to missing connectionMonitorInterval"
      );
    }

    const rtcClient = new RtcClient({
      lanOnlyMode: true,
      receive: this.handleMessage,
      sessionType,
    });

    await rtcClient.connectLan(this.peerUrl);

    // WebRTC requires a signaling phase when forming a new connection.
    // Wait for the signaling process to complete...
    while (rtcClient.getConnectionStatus(this.peerUrl) !== "connected") {
      await delay(100);
    }
    this.rtcClient = rtcClient;
    this.initConnectionMonitoring();
  }

  private initConnectionMonitoring() {
    this.connectionMonitorInterval = setInterval(async () => {
      let dataChannelClosed = false;

      if (this.rtcClient) {
        if (this.rtcClient.getConnectionStatus(this.peerUrl) !== "connected") {
          console.debug(`${new Date().toISOString()} :: data channel closed`);
          dataChannelClosed = true;
        }
      }

      if (!this.rtcClient || dataChannelClosed) {
        this.emit("disconnect");
        this.stopRealtimeConnection().catch((err) => {
          console.error(err);
        });
      }
    }, 1000);
  }

  async getRemotePeer(): Promise<IRtcPeer> {
    return {
      id: this.peerUrl,
      organizationId: "",
      deviceId: this.id,
      capabilities: [],
      capabilitySet: {},
    };
  }

  async stopRealtimeConnection() {
    let throwNotStartedError = false;

    if (this.rtcClient) {
      this.stopConnectionMonitoring();

      if (this.id) {
        await this.rtcClient.disconnect(this.id);
        this.remoteDevicePeerId = null;
      } else {
        throwNotStartedError = true;
      }

      try {
        await this.rtcClient.shutdown();
      } finally {
        this.rtcClient = undefined;
      }
    }

    if (throwNotStartedError) {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  async getCommandMetadata(): Promise<{ commands: any[] }> {
    return (await fetch(`${this.peerUrl}/v1/commands`)).json();
  }

  async sendCommand(
    name: string,
    data?: string,
    time?: Date,
    metadata?: {}
  ): Promise<Response> {
    const parameter = {
      value: data,
      scrubberTime: (time || new Date()).toISOString(),
      meta: metadata,
    };

    const res = await fetch(`${this.peerUrl}/v1/enqueue-command`, {
      method: "POST",
      body: JSON.stringify({
        command: name,
        parameter: parameter,
      }),
      headers: {
        "Content-Type": "application/json",
      },
    });

    return res;
  }
}
