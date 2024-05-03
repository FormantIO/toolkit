import {
  IRtcSendConfiguration,
  IRtcStreamMessage,
  RtcClient,
  IRtcPeer,
} from "@formant/realtime-sdk";
import { DataChannel } from "../DataChannel";
import { EventEmitter } from "eventemitter3";
import { Manipulator } from "../Manipulator";
import {
  BinaryRequestDataChannel,
  TextRequestDataChannel,
} from "../RequestDataChannel";
import { defined } from "../../../common/defined";
import { IRealtimeSubscriber } from "./IRealtimeSubscriber";
import { ICustomDataChannelCreator } from "./ICustomDataChannelCreator";
import {
  ConfigurationDocument,
  RealtimeListener,
  RealtimeAudioStream,
  RealtimeVideoStream,
  RealtimeDataStream,
} from "./device.types";
import { TelemetryResult } from "../model/TelemetryResult";
import { IEventQuery } from "../model/IEventQuery";
import { IEvent } from "../model/IEvent";

export abstract class BaseDevice
  extends EventEmitter
  implements IRealtimeSubscriber, ICustomDataChannelCreator
{
  rtcClient: RtcClient | undefined;
  remoteDevicePeerId: string | null = null;

  protected realtimeListeners: RealtimeListener[] = [];

  protected connectionMonitorInterval:
    | ReturnType<typeof setInterval>
    | undefined;

  abstract getConfiguration(): Promise<ConfigurationDocument>;
  abstract startRealtimeConnection(sessionType?: number): Promise<void>;
  abstract getRemotePeer(): Promise<IRtcPeer>;
  abstract getTelemetry(
    streamNameOrStreamNames: string | string[],
    start: Date,
    end: Date,
    tags?: { [key in string]: string[] },
    limit?: number,
    offset?: number,
    latestOnly?: boolean
  ): Promise<TelemetryResult[]>;
  abstract queryEvents(query: IEventQuery): Promise<IEvent[]>;

  protected handleMessage = (peerId: string, message: any) => {
    this.realtimeListeners.forEach((_) => _(peerId, message));
  };

  protected stopConnectionMonitoring() {
    clearInterval(this.connectionMonitorInterval);
    this.connectionMonitorInterval = undefined;
  }

  protected assertNotCancelled(cancelled: boolean): void {
    if (cancelled) throw new Error("Cancelled by deadline");
  }

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

  async getRealtimeVideoStreams(): Promise<RealtimeVideoStream[]> {
    const document = (await this.getConfiguration()) as any;
    const streams: { name: string }[] = [];

    for (const _ of document.teleop?.hardwareStreams ?? []) {
      if (_.rtcStreamType === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    for (const _ of document.teleop?.rosStreams ?? []) {
      if (_.topicType == "formant/H264VideoFrame") {
        streams.push({
          name: _.topicName,
        });
      }
      if (
        (_.topicType === "sensor_msgs/Image" ||
          _.topicType === "sensor_msgs/CompressedImage") &&
        _.encodeVideo
      ) {
        streams.push({
          name: _.topicName,
        });
      }
    }
    for (const _ of document.teleop?.customStreams ?? []) {
      if (_.rtcStreamType === "h264-video-frame") {
        streams.push({
          name: _.name,
        });
      }
    }
    return streams;
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

  async changeStreamAudioType(streamName: string, newFormat: "wav" | "opus") {
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName,
      setAudioFormat: newFormat,
    });
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

  async getRealtimeAudioStreams(): Promise<RealtimeAudioStream[]> {
    const document = await this.getConfiguration();
    const streams: { name: string }[] = [];

    for (const _ of document.teleop?.hardwareStreams ?? []) {
      if (_.rtcStreamType === "audio-chunk") {
        streams.push({
          name: _.name,
        });
      }
    }
    for (const _ of document.teleop?.rosStreams ?? []) {
      if (_.topicType == "audio_common_msgs/AudioData") {
        streams.push({
          name: _.topicName,
        });
      }
    }
    for (const _ of document.teleop?.customStreams ?? []) {
      if (_.rtcStreamType === "audio-chunk") {
        streams.push({
          name: _.name,
        });
      }
    }
    return streams;
  }
}
