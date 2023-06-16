import {
  IRtcSendConfiguration,
  IRtcStreamMessage,
  IRtcStreamPayload,
  RtcClient,
} from "@formant/realtime-sdk";
import { ITags } from "./model/ITags";
import { Uuid } from "./model/Uuid";
import { SessionType } from "./model/SessionType";
import { DataChannel } from "./DataChannel";
import { RtcStreamType } from "@formant/realtime-sdk/dist/model/RtcStreamType";
import { EventEmitter } from "events";
import { Manipulator } from "./Manipulator";
import { BinaryRequestDataChannel, TextRequestDataChannel } from "./main";
import { defined } from "../../common/defined";
import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";

export interface ConfigurationDocument {
  tags: ITags;
  urdfFiles: string[];
  telemetry?: {
    streams?: { name: string; disabled?: boolean; onDemand?: boolean }[];
  };
  teleop?: {
    customStreams?: {
      name: string;
      rtcStreamType: RtcStreamType;
    }[];
    hardwareStreams?: {
      name: string;
      rtcStreamType: RtcStreamType;
    }[];
    rosStreams?: {
      rtcStreamType: RtcStreamType;
      topicName: string;
      topicType: string;
    }[];
  };
  adapters?: IAdapterConfiguration[];
  application?: { configurationMap: { [key: string]: string } };
}

export interface Command {
  id: string;
  name: string;
  command: string;
  description: string;
  parameterEnabled: true;
  parameterValue: string | null;
  tags: ITags;
  parameterMeta?: {
    topic?: string;
  };
}

export interface IJointState {
  name: string[];
  position: number[];
  velocity?: number[];
  effort?: number[];
}

export interface TelemetryStream {
  name: string;
  onDemand: boolean;
}

export interface IAdapterConfiguration {
  id: Uuid;
  name: string;
  fileId: Uuid;
  execCommand: string;
  configuration?: string;
}

export type RealtimeMessage = {
  header: {
    created: number;
    stream: {
      entityId: string;
      streamName: string;
      streamType: RtcStreamType;
    };
  };
  payload: IRtcStreamPayload;
};

export type RealtimeListener = (
  peerId: string,
  message: RealtimeMessage
) => void;

export type RealtimeAudioStream = {
  name: string;
};

export type RealtimeVideoStream = {
  name: string;
};

export type RealtimeDataStream = {
  name: string;
};

export interface IStartRealtimeConnectionOptions {
  sessionType?: SessionType;
  deadlineMs?: number;
  maxConnectRetries?: number;
}

export abstract class BaseDevice extends EventEmitter {
  rtcClient: RtcClient | undefined;
  remoteDevicePeerId: string | null = null;

  protected realtimeListeners: RealtimeListener[] = [];

  protected connectionMonitorInterval: NodeJS.Timeout | undefined;

  abstract getConfiguration(): Promise<ConfigurationDocument>;
  abstract startRealtimeConnection(sessionType?: number): Promise<void>;
  abstract getRemotePeer(): Promise<IRtcPeer>;

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
