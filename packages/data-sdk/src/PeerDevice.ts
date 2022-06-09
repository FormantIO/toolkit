import { IRtcConnectConfiguration, RtcClient } from "@formant/realtime-sdk";
import { delay } from "../../common/delay";
import { defined } from "../../common/defined";
import { DataChannel } from "./DataChannel";
import { Manipulator } from "./Manipulator";
import {
  TextRequestDataChannel,
  BinaryRequestDataChannel,
} from "./RequestDataChannel";
import {
  ConfigurationDocument,
  IRealtimeDevice,
  RealtimeDataStream,
  RealtimeListener,
  RealtimeVideoStream,
} from "./Device";
import { IStreamCurrentValue } from "./model/IStreamCurrentValue";
import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";

export class PeerDevice implements IRealtimeDevice {
  rtcClient: RtcClient | undefined;
  remoteDevicePeerId: string | undefined;

  realtimeListeners: RealtimeListener[] = [];
  id!: string;
  constructor(public peerUrl: string) {}

  async getLatestTelemetry(): Promise<IStreamCurrentValue[]> {
    const data = await fetch(`${this.peerUrl}/telemetry`);
    const telemetry = (await data.json()) as {
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

  async getDeviceId(): Promise<string> {
    let result = await fetch(`${this.peerUrl}/configuration`);
    const cfg = await result.json();
    return cfg.agent_config.id;
  }

  async getConfiguration(): Promise<ConfigurationDocument> {
    let result = await fetch(`${this.peerUrl}/configuration`);
    const cfg = await result.json();
    return cfg.agent_config.document;
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

  async startRealtimeConnection(_config?: IRtcConnectConfiguration) {
    if (!this.rtcClient) {
      const rtcClient = new RtcClient({
        lanOnlyMode: true,
        receive: this.handleMessage,
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

    for (const _ of document.teleop.hardwareStreams ?? []) {
      if (_.rtcStreamType === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    for (const _ of document.teleop.rosStreams ?? []) {
      if (_.topicType == "formant/H264VideoFrame") {
        streams.push({
          name: _.topicName,
        });
      }
    }
    for (const _ of document.teleop.customStreams ?? []) {
      if (_.rtcStreamType === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    return streams;
  }

  async getRealtimeManipulators(): Promise<Manipulator[]> {
    const document = (await this.getConfiguration()) as any;
    const manipulators = [];

    for (const _ of document.teleop.rosStreams ?? []) {
      if (_.topicType == "sensor_msgs/JointState") {
        manipulators.push(
          new Manipulator(this, {
            currentJointStateStream: { name: _.topicName },
            plannedJointStateStream: _.plannedTopic
              ? { name: _.plannedTopic }
              : undefined,
            planValidStream: _.planValidTopic
              ? { name: _.planValidTopic }
              : undefined,
            endEffectorStream: _.endEffectorTopic
              ? { name: _.endEffectorTopic }
              : undefined,
            endEffectorLinkName: _.endEffectorLinkName,
            baseReferenceFrame: _.baseReferenceFrame,
            localFrame: _.localFrame,
          })
        );
      }
    }
    return manipulators;
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

  createCustomRequestDataChannel(
    channelName: string,
    timeout: number = 3000 // 3 seconds default timeout
  ): TextRequestDataChannel {
    return new TextRequestDataChannel(this, channelName, timeout);
  }

  createCustomBinaryRequestDataChannel(
    channelName: string,
    timeout: number = 3000 // 3 seconds default timeout
  ): BinaryRequestDataChannel {
    return new BinaryRequestDataChannel(this, channelName, timeout);
  }
}
