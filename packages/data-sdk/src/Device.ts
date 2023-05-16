import { EventEmitter } from "events";
import {
  IRtcSendConfiguration,
  IRtcStreamMessage,
  IRtcStreamPayload,
  RtcClient,
  RtcClientV1,
  RtcSignalingClient,
  SignalingPromiseClient,
} from "@formant/realtime-sdk";
import { RtcStreamType } from "@formant/realtime-sdk/dist/model/RtcStreamType";

import { FORMANT_API_URL } from "./config";
import { delay } from "../../common/delay";
import { defined } from "../../common/defined";
import { Authentication } from "./Authentication";
import { DataChannel } from "./DataChannel";
import { CaptureStream } from "./CaptureStream";
import { Manipulator } from "./Manipulator";
import { Fleet } from "./Fleet";
import {
  TextRequestDataChannel,
  BinaryRequestDataChannel,
} from "./RequestDataChannel";
import { Uuid } from "./model/Uuid";
import { InterventionType } from "./model/InterventionType";
import { IInterventionTypeMap } from "./model/IInterventionTypeMap";
import { IInterventionResponse } from "./model/IInterventionResponse";
import { IEventQuery } from "./model/IEventQuery";
import { AggregateLevel } from "./model/AggregateLevel";
import { EventType } from "./model/EventType";
import { IShare } from "./model/IShare";
import { isRtcPeer } from "./main";

// get query param for "rtc_client"
const urlParams = new URLSearchParams(window.location.search);
const rtcClientVersion = urlParams.get("rtc_client");

export interface ConfigurationDocument {
  tags: { [key: string]: string };
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

export const SessionType = {
  Teleop: 1,
  Observe: 3,
};

export interface IRealtimeDevice {
  startRealtimeConnection(sessionType?: number): Promise<void>;
  startListeningToRealtimeDataStream(stream: RealtimeDataStream): Promise<void>;
  stopListeningToRealtimeDataStream(stream: RealtimeDataStream): Promise<void>;
  addRealtimeListener(listener: RealtimeListener): void;
  removeRealtimeListener(listener: RealtimeListener): void;
  createCustomDataChannel(
    channelName: string,
    rtcConfig?: RTCDataChannelInit
  ): Promise<DataChannel>;
}

export class Device extends EventEmitter implements IRealtimeDevice {
  rtcClient: RtcClient | RtcClientV1 | undefined;
  remoteDevicePeerId: string | undefined;

  realtimeListeners: RealtimeListener[] = [];

  private connectionMonitorInterval: NodeJS.Timeout | undefined;

  constructor(
    public id: string,
    public name: string,
    private organizationId: string
  ) {
    super();
  }

  async getLatestTelemetry() {
    const data = await fetch(
      `${FORMANT_API_URL}/v1/queries/stream-current-value`,
      {
        method: "POST",
        body: JSON.stringify({
          deviceIds: [this.id],
        }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const telemetry = await data.json();
    return telemetry.items;
  }

  async getConfiguration(): Promise<ConfigurationDocument> {
    let result = await fetch(`${FORMANT_API_URL}/v1/admin/devices/${this.id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const device = await result.json();
    if (!device.state.reportedConfiguration) {
      throw new Error(
        "Device has no configuration, has it ever been turned on?"
      );
    }
    const version = device.state.reportedConfiguration.version;
    result = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${this.id}/configurations/${version}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const config = await result.json();
    return config.document;
  }

  async getFileUrl(fileId: string): Promise<string[]> {
    const result = await fetch(`${FORMANT_API_URL}/v1/admin/files/query`, {
      method: "POST",
      body: JSON.stringify({
        fileIds: [fileId],
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const files = await result.json();
    return files.fileUrls;
  }

  private handleMessage = (peerId: string, message: any) => {
    this.realtimeListeners.forEach((_) => _(peerId, message));
  };

  getRealtimeStatus(): "disconnected" | "connecting" | "connected" {
    if (this.rtcClient && this.remoteDevicePeerId) {
      return this.rtcClient.getConnectionStatus(this.remoteDevicePeerId);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  getRealtimePing(): number | undefined {
    if (this.rtcClient && this.remoteDevicePeerId) {
      return this.rtcClient.getPing(this.remoteDevicePeerId);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  /**
   * Starts a real-time connection with the remote device using WebRTC.
   * @param {number} [sessionType] - Optional session type to be used for the connection.
   * @throws `Error`  If the connection could not be established or if a connection already exists.
   * @returns {void}
   */

  async startRealtimeConnection(sessionType?: number): Promise<void> {
    if (!this.rtcClient || this.connectionMonitorInterval === undefined) {
      let rtcClient;

      if (rtcClientVersion === "1") {
        rtcClient = new RtcClientV1({
          signalingClient: new RtcSignalingClient(
            FORMANT_API_URL + "/v1/signaling"
          ),
          getToken: async () =>
            defined(
              Authentication.token,
              "Realtime when user isn't authorized"
            ),
          receive: this.handleMessage,
        });
      } else {
        rtcClient = new RtcClient({
          signalingClient: new SignalingPromiseClient(
            FORMANT_API_URL,
            null,
            null
          ),
          getToken: async () => {
            return defined(
              Authentication.token,
              "Realtime when user isn't authorized"
            );
          },
          receive: this.handleMessage,
          sessionType: sessionType,
        });
      }

      if ((rtcClient as any).isReady) {
        while (!(rtcClient as RtcClient).isReady()) {
          await delay(100);
        }
      }

      // WebRTC requires a signaling phase when forming a new connection.

      this.remoteDevicePeerId = await this.getRemoteDevicePeerId(rtcClient);

      const sessionId = await this.createSession(rtcClient);

      // Wait for the signaling process to complete...

      if (!!sessionId) {
        const tries = 5;
        for (let i = 0; i < tries; i++) {
          const connectionCompleted =
            rtcClient.getConnectionStatus(this.remoteDevicePeerId) ===
            "connected";
          if (connectionCompleted) {
            this.initConnectionMonitoring();
            this.rtcClient = rtcClient;
            this.emit("connect");
            return;
          }
          await delay(100);
        }
        throw new Error(
          "A session was created, but the connection could not be established, possibly due to network issues or misconfigured settings."
        );
      } else {
        throw new Error(`Unable to establish a connection at this time.`);
      }
    } else {
      throw new Error(
        `Already created realtime connection to device ${this.id}`
      );
    }
  }

  private async getRemoteDevicePeerId(rtcClient: RtcClient | RtcClientV1) {
    // Each online device and user has a peer in the system
    const peers = await rtcClient.getPeers();

    // Find the device peer corresponding to the device's ID
    const devicePeer = peers.find((_) => _.deviceId === this.id);

    if (!isRtcPeer(devicePeer)) {
      // If the device is offline, we won't be able to find its peer.
      throw new Error("Cannot find peer, is the robot offline?");
    }
    return devicePeer.id;
  }

  private async createSession(rtcClient: RtcClient | RtcClientV1) {
    // We can connect our real-time communication client to device peers by their ID
    const tries = 3;
    if (this.remoteDevicePeerId) {
      for (let i = 0; i < tries; i++) {
        const connectionId = await (rtcClient as RtcClient).connect(
          this.remoteDevicePeerId
        );
        if (!!connectionId) {
          return connectionId;
        }
        delay(100);
      }
    }
    return;
  }

  private initConnectionMonitoring() {
    this.connectionMonitorInterval = setInterval(async () => {
      if (!this.isV2Signaling(this.rtcClient)) {
        return;
      }

      const rtcConnections = this.rtcClient.getConnections();
      const connection = rtcConnections.find(
        (_) => _.getRemotePeerId() === this.remoteDevicePeerId && _.isActive()
      );
      let dataChannelClosed = false;
      if (connection === undefined || !connection.isReady()) {
        console.debug(`${new Date().toISOString()} :: data channel closed`);
        dataChannelClosed = true;
      } else {
        console.debug(
          `${new Date().toISOString()} :: data channel appears active`
        );
      }

      if (
        !this.rtcClient ||
        !this.remoteDevicePeerId ||
        (await this.rtcClient.getConnectionStatsInfo(
          this.remoteDevicePeerId
        )) === undefined ||
        dataChannelClosed
      ) {
        this.emit("disconnect");
        this.stopRealtimeConnection().catch((err) => {
          console.error(err);
        });
      }
    }, 1000);
  }

  private isV2Signaling(
    rtcClient: RtcClient | RtcClientV1 | undefined
  ): rtcClient is RtcClient {
    return (rtcClient as RtcClient).getConnectionStatsInfo !== undefined;
  }

  private stopConnectionMonitoring() {
    clearInterval(this.connectionMonitorInterval);
    this.connectionMonitorInterval = undefined;
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
    console.debug(`RealtimeVideo(${this.id}).start(${stream.name})`);
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
    console.debug(`RealtimeVideo(${this.id}).stop(${stream.name})`);
  }

  async startListeningToRealtimeDataStream(stream: RealtimeDataStream) {
    console.debug(`RealtimeDataStream(${this.id}).start(${stream.name})`);
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
    console.debug(`RealtimeDataStream(${this.id}).stop(${stream.name})`);
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

  async getRemotePeer() {
    // Each online device and user has a peer in the system
    const peers = await defined(
      this.rtcClient,
      "Realtime connection has not been started"
    ).getPeers();

    // Find the device peer corresponding to the device's ID
    const devicePeer = peers.find((_) => _.deviceId === this.id);
    return defined(
      devicePeer,
      "Could not find remote peer for device " + this.id
    );
  }

  async stopRealtimeConnection() {
    if (this.rtcClient && this.remoteDevicePeerId) {
      this.stopConnectionMonitoring();
      await this.rtcClient.disconnect(this.remoteDevicePeerId);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  async isInRealtimeSession(): Promise<boolean> {
    let peers = await Fleet.getPeers();
    let sessions = await Fleet.getRealtimeSessions();
    let peer = peers.find((_) => _.deviceId === this.id);
    if (peer) {
      return sessions[peer.id].length > 0;
    }
    return false;
  }

  async getAvailableCommands(): Promise<Command[]> {
    const result = await fetch(
      `${FORMANT_API_URL}/v1/admin/command-templates/`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const commands = await result.json();
    return commands.items.map((i: any) => ({
      name: i.name,
      id: i.id,
      command: i.command,
      description: i.description,
      parameterEnabled: i.parameterEnabled,
      parameterValue: i.parameterValue,
      parameterMeta: i.parameterMeta,
      enabled: i.enabled,
    }));
  }

  async sendCommand(
    name: string,
    data?: string,
    time?: Date,
    metadata?: {}
  ): Promise<Response> {
    const commands = await this.getAvailableCommands();
    const command = commands.find((_) => _.name === name);
    if (!command) {
      throw new Error(`Could not find command with name "${name}"`);
    }

    let d: string = "";

    if (data === undefined) {
      if (command.parameterEnabled && command.parameterValue) {
        d = command.parameterValue;
      }
    } else {
      d = data;
    }

    let parameter = {
      value: d,
      scrubberTime: (time || new Date()).toISOString(),
      meta: {
        ...command.parameterMeta,
        ...metadata,
      },
    };

    const res = await fetch(`${FORMANT_API_URL}/v1/admin/commands`, {
      method: "POST",
      body: JSON.stringify({
        commandTemplateId: command.id,
        organizationId: this.organizationId,
        deviceId: this.id,
        command: command.command,
        parameter: parameter,
        userId: Authentication.currentUser?.id,
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });

    return res;
  }

  async getCommand(id: string): Promise<Response> {
    const res = await fetch(`${FORMANT_API_URL}/v1/admin/commands/${id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return res;
  }

  async createCustomDataChannel(
    channelName: string,
    rtcConfig?: RTCDataChannelInit
  ): Promise<DataChannel> {
    if (rtcClientVersion === "1") {
      throw new Error(
        "createCustomDataChannel is not supported in rtcClientVersion 1"
      );
    }
    const client = defined(
      this.rtcClient,
      "Realtime connection has not been started"
    );

    const devicePeer = await this.getRemotePeer();
    const p = await new Promise<DataChannel>((resolve) => {
      (client as RtcClient).createCustomDataChannel(
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

  async createCaptureStream(streamName: string) {
    const result = await fetch(`${FORMANT_API_URL}/v1/admin/capture-sessions`, {
      method: "POST",
      body: JSON.stringify({
        deviceId: this.id,
        streamName,
        tags: {},
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const captureSession = await result.json();
    return new CaptureStream(captureSession);
  }

  async getTelemetry(
    streamNameOrStreamNames: string | string[],
    start: Date,
    end: Date,
    tags?: { [key in string]: string[] }
  ) {
    return await Fleet.getTelemetry(
      this.id,
      streamNameOrStreamNames,
      start,
      end,
      tags
    );
  }

  async getTelemetryStreams(): Promise<TelemetryStream[]> {
    const config = await this.getConfiguration();

    const result = await fetch(
      `${FORMANT_API_URL}/v1/queries/metadata/stream-names`,
      {
        method: "POST",
        body: JSON.stringify({
          deviceIds: [this.id],
        }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const disabledList: string[] = [];
    const onDemandList: string[] = [];
    config.telemetry?.streams?.forEach((_) => {
      if (_.disabled !== true) {
        disabledList.push(_.name);
      }
      if (_.onDemand === true) {
        onDemandList.push(_.name);
      }
    });
    console.log(onDemandList);

    const data = await result.json();

    let streamNames = (data.items as string[])
      .filter((_) => !disabledList.includes(_))
      .map((_) => ({ name: _, onDemand: onDemandList.includes(_) }));

    return streamNames;
  }

  async createInterventionRequest<T extends InterventionType>(
    message: string,
    interventionType: InterventionType,
    interventionRequest: IInterventionTypeMap[T]["request"],
    tags?: { [key in string]: string[] }
  ): Promise<(id: string) => IInterventionTypeMap[T]["response"]> {
    const intervention = await fetch(
      `${FORMANT_API_URL}/v1/admin/intervention-requests`,
      {
        method: "POST",
        body: JSON.stringify({
          message,
          interventionType,
          time: new Date().toISOString(),
          deviceId: this.id,
          tags,
          data: interventionRequest,
        }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const interventionJson = await intervention.json();

    return interventionJson;
  }

  async addInterventionResponse<T extends InterventionType>(
    interventionId: string,
    interventionType: InterventionType,
    data: IInterventionTypeMap[T]["response"]
  ): Promise<IInterventionResponse> {
    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/intervention-responses`,
      {
        method: "POST",
        body: JSON.stringify({
          interventionId,
          interventionType,
          data,
        }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const interventionResponse = await response.json();
    return interventionResponse;
  }

  async getAnnotationCount(query: IEventQuery, tagKey: string) {
    return await Fleet.getAnnotationCount(
      { ...query, deviceIds: [this.id] },
      tagKey
    );
  }

  async getAnnotationCountByIntervals(
    query: IEventQuery,
    tagKey: string,
    aggregate: AggregateLevel
  ) {
    return await Fleet.getAnnotationCountByIntervals(
      { ...query, deviceIds: [this.id] },
      tagKey,
      aggregate
    );
  }

  async eventsCounter(
    eventTypes: EventType[],
    timeFrame: AggregateLevel,
    range: number,
    time: number,
    query?: IEventQuery
  ) {
    return await Fleet.eventsCounter(eventTypes, timeFrame, range, time, {
      ...query,
      deviceIds: [this.id],
    });
  }

  async createShareLink(share: IShare, view: string) {
    share.scope.deviceIds = [this.id];
    return await Fleet.createShareLink(share, view);
  }
}
