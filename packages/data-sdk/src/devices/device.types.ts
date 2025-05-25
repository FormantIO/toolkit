import { IRtcStreamPayload, RtcStreamType } from "@formant/realtime-sdk";
import { IDictionary } from "../model/IDictionary";
import { IFilter } from "../model/IFilter";
import { IScopeFilter } from "../model/IScopeFilter";
import { ITaggedEntity } from "../model/ITaggedEntity";
import { ITags } from "../model/ITags";
import { SessionType } from "../model/SessionType";
import { Uuid } from "../model/Uuid";

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
export interface ICommandDeliverySettings {
  // command is dropped after it expires (defaults to defaultCommandTtlMs)
  ttlMs?: number;
  // `false` (default) for fire-and-forget (at most once delivery)
  // `true` for at least once delivery
  retryable?: boolean;

  // TODO?
  // maxRetries?: number;
  // retryDelayMs?: number;
  // parallelExecution?: boolean;
  // maxPendingCommands?: number;
}

export interface ICommandTemplate extends ITaggedEntity {
  organizationId?: Uuid;
  name: string;
  command: string;
  description?: string;
  parameterEnabled: boolean;
  allowParameterOverride?: boolean;
  parameterValue?: string | null;
  parameterMeta?: IDictionary;
  deviceScope?: IScopeFilter;
  enabled?: boolean;
  deviceFilter: IFilter | null;
  lambdaUri?: string | null;
  deliverySettings?: ICommandDeliverySettings;
  schema?: string | null;
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
