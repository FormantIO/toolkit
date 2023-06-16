import { IRtcStreamPayload } from "@formant/realtime-sdk";
import { ITags } from "./model/ITags";
import { Uuid } from "./model/Uuid";
import { SessionType } from "./model/SessionType";
import { DataChannel } from "./DataChannel";
import { RtcStreamType } from "@formant/realtime-sdk/dist/model/RtcStreamType";
import { EventEmitter } from "events";

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
}

export interface Command {
  id: string;
  name: string;
  command: string;
  description: string;
  parameterEnabled: true;
  parameterValue: string | null;
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
  abstract startRealtimeConnection(sessionType?: number): Promise<void>;
  abstract startListeningToRealtimeDataStream(
    stream: RealtimeDataStream
  ): Promise<void>;
  abstract stopListeningToRealtimeDataStream(
    stream: RealtimeDataStream
  ): Promise<void>;
  abstract addRealtimeListener(listener: RealtimeListener): void;
  abstract removeRealtimeListener(listener: RealtimeListener): void;
  abstract createCustomDataChannel(
    channelName: string,
    rtcConfig?: RTCDataChannelInit
  ): Promise<DataChannel>;

  protected assertNotCancelled(cancelled: boolean): void {
    if (cancelled) throw new Error("Cancelled by deadline");
  }
}
