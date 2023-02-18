import { RtcClient } from "@formant/realtime-sdk";
import { delay } from "./delay";
import { defined } from "./defined";
import { DataChannel } from "./DataChannel";
import {
  ConfigurationDocument,
  IRealtimeDevice,
  RealtimeDataStream,
  RealtimeListener,
  RealtimeVideoStream,
} from "@formant/data-sdk/Device";
import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";
import { IRtcSendConfiguration, IRtcStreamMessage } from "@formant/data-sdk";

export class PeerDevice implements IRealtimeDevice {
  rtcClient: RtcClient | undefined;
  remoteDevicePeerId: string | undefined;

  realtimeListeners: RealtimeListener[] = [];
  id!: string;
  constructor(public peerUrl: string) {}

  async getLatestTelemetry(): Promise<any[]> {
    const data = await fetch(`${this.peerUrl}/telemetry`);
    const telemetry = (await data.json()) as {
      [key in string]: { timestamp: string };
    };
    const entries = Object.entries(telemetry);
    return entries.map(([stream, latestValue]) => {
      const v = {
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

  async getDeviceId(): Promise<string> {
    let result = await fetch(`${this.peerUrl}/configuration`);
    const cfg = await result.json();
    return cfg.configuration.id;
  }

  async getConfiguration(): Promise<ConfigurationDocument> {
    let result = await fetch(`${this.peerUrl}/configuration`);
    const cfg = await result.json();
    console.log(cfg)
    return cfg.configuration.document;
  }

  private handleMessage = (peerId: string, message: any) => {
    this.realtimeListeners.forEach((_) => _(peerId, message));
  };

  getRealtimeStatus(): "disconnected" | "connecting" | "connected" {
    if (this.rtcClient && this.remoteDevicePeerId) {
      return this.rtcClient.getConnectionStatus(this.remoteDevicePeerId);
    } else {
      throw new Error(`Realtime connection hasn't been started`);
    }
  }

  getRealtimePing(): number | undefined {
    if (this.rtcClient && this.remoteDevicePeerId) {
      return this.rtcClient.getPing(this.remoteDevicePeerId);
    } else {
      throw new Error(`Realtime connection hasn't been started`);
    }
  }

  async startRealtimeConnection(sessionType?: number) {
    if (!this.rtcClient) {
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
    } else {
      throw new Error(
        `Already created realtime connection to device ${this.id}`
      );
    }
  }

  addRealtimeListener(listener: RealtimeListener) {
    this.realtimeListeners.push(listener);
  }

  removeRealtimeListener(listener: RealtimeListener) {
    const i = this.realtimeListeners.indexOf(listener);
    if (i === -1) {
      throw new Error("Could not find realtime listener to remove");
    }
    this.realtimeListeners.splice(i, 1);
  }

  async getRealtimeVideoStreams(): Promise<RealtimeVideoStream[]> {
    const document = (await this.getConfiguration()) as any;
    const streams = [];
    console.log(document)

    for (const _ of document.teleop.hardware_streams ?? []) {
        console.log(_)
      if (_.rtc_stream_type === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    for (const _ of document.teleop.ros_streams ?? []) {
      if (_.topicType == "formant/H264VideoFrame") {
        streams.push({
          name: _.topic_name,
        });
      }
    }
    for (const _ of document.teleop.custom_streams ?? []) {
      if (_.rtc_stream_type === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    console.log(streams)
    return streams;
  }

  async getRealtimeTwistStreams(): Promise<RealtimeVideoStream[]> {
    const document = (await this.getConfiguration()) as any;
    const streams = [];
    console.log("twist")
    console.log(document)
    for (const _ of document.teleop.hardware_streams ?? []) {
        console.log(_)
      if (_.rtc_stream_type === "twist") {
        streams.push({
          name: _.name,
        });
      }
    }
    for (const _ of document.teleop.ros_streams ?? []) {
        console.log(_)
      if (_.topic_type == "GEOMETRY_MSGS_TWIST") {
        streams.push({
          name: _.topic_name,
        });
      }
    }
    for (const _ of document.teleop.custom_streams ?? []) {
        console.log(_)

      if (_.rtc_stream_type === "twist") {
        streams.push({
          name: _.name,
        });
      }
    }
    console.log(streams)
    return streams;
  }
  async startListeningToRealtimeVideo(stream: RealtimeVideoStream) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: true,
      pipeline: "rtc",
    });
  }

  async stopListeningToRealtimeVideo(stream: RealtimeVideoStream) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );
    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: false,
      pipeline: "rtc",
    });
  }

  async startListeningToRealtimeDataStream(stream: RealtimeDataStream) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: true,
      pipeline: "rtc",
    });
  }

  async stopListeningToRealtimeDataStream(stream: RealtimeDataStream) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );
    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: false,
      pipeline: "rtc",
    });
  }

  async enableRealtimeTelemetryPriorityIngestion(streamName: string) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: streamName,
      enablePriorityUpload: true,
      pipeline: "telemetry",
    });
  }

  async disableRealtimeTelemetryPriorityIngestion(streamName: string) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: streamName,
      enablePriorityUpload: false,
      pipeline: "telemetry",
    });
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
    if (this.rtcClient) {
      await this.rtcClient.disconnect(this.id);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  async createCustomDataChannel(
    channelName: string,
    rtcConfig?: RTCDataChannelInit
  ): Promise<DataChannel> {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    const p = await new Promise<DataChannel>((resolve) => {
      client.createCustomDataChannel(
        defined(devicePeer).id,
        channelName,
        {
          ordered: true,
          ...rtcConfig,
        },
        false,
        (_peerId, channel) => {
          const dataChannel = new DataChannel(channel);
          resolve(dataChannel);
        }
      );
    });
    await p.waitTilReady();
    return p;
  }

  async sendRealtimeMessage(
    message: IRtcStreamMessage,
    config: IRtcSendConfiguration = {
      channelLabel: "stream.reliable",
    }
  ) {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.send(defined(devicePeer).id, message, config);
  }
}
