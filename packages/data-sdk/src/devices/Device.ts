import { RtcClient } from "@formant/realtime-sdk";
import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";
import { getRtcClientPool } from "../AppRtcClientPools";
import { Authentication } from "../Authentication";
import { CaptureStream } from "../CaptureStream";
import { FORMANT_API_URL } from "../config";
import { delay } from "../../../common/delay";
import { defined } from "../../../common/defined";
import { InterventionType } from "../model/InterventionType";
import { IInterventionTypeMap } from "../model/IInterventionTypeMap";
import { IInterventionResponse } from "../model/IInterventionResponse";
import { IEventQuery } from "../model/IEventQuery";
import { AggregateLevel } from "../model/AggregateLevel";
import { EventType } from "../model/EventType";
import { IShare } from "../model/IShare";
import { SessionType } from "../model/SessionType";
import { isRtcPeer } from "../utils/isRtcPeer";
import {
  Command,
  ConfigurationDocument,
  IStartRealtimeConnectionOptions,
  TelemetryStream,
} from "./device.types";
import { BaseDevice } from "./BaseDevice";
import { ITags } from "../model/ITags";
import { createShareLink } from "../api/createShareLink";
import { eventsCounter } from "../api/eventsCounter";
import { getAnnotationCount } from "../api/getAnnotationCount";
import { getAnnotationCountByIntervals } from "../api/getAnnotationCountByIntervals";
import { getTelemetry } from "../api/getTelemetry";
import { getRealtimeSessions } from "../api/getRealtimeSessions";
import { getPeers } from "../api/getPeers";
import { createDevice } from "../api/createDevice";
import { patchDevice } from "../api/patchDevice";
import { getDevicesData } from "../api/getDevicesData";
import { queryDevicesData } from "../api/queryDevicesData";

export class Device extends BaseDevice {
  constructor(
    public id: string,
    public name: string,
    private organizationId: string,
    public tags?: ITags
  ) {
    super();
  }

  static createDevice = createDevice;
  static patchDevice = patchDevice;
  static getDevicesData = getDevicesData;
  static queryDevicesData = queryDevicesData;

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

  /**
   * Starts a real-time connection with the remote device using WebRTC.
   * @param {number} [options] - Optional session type to be used for the connection.
   * @throws `Error`  If the connection could not be established or if a connection already exists.
   * @returns {void}
   */
  async startRealtimeConnection(
    options: SessionType | IStartRealtimeConnectionOptions = {}
  ): Promise<void> {
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

    const {
      sessionType,
      deadlineMs = 10_000,
      maxConnectRetries = 3,
    } = typeof options === "number"
      ? ({ sessionType: options } as IStartRealtimeConnectionOptions)
      : options;

    const pool = getRtcClientPool({
      sessionType,
    });
    const rtcClient = pool.get(this.handleMessage);

    let cancelled = false;
    const deadlinePromise = new Promise<never>((_, reject) =>
      setTimeout(() => {
        cancelled = true;
        reject(
          new Error(
            "Connection timed out: the connection could not be finalized in time, " +
              "possibly due to network issues or misconfigured settings."
          )
        );
      }, deadlineMs)
    );

    const establishConnection = async (): Promise<string> => {
      if ("isReady" in rtcClient) {
        while (!rtcClient.isReady()) {
          this.assertNotCancelled(cancelled);
          await delay(100);
        }
      }

      // WebRTC requires a signaling phase when forming a new connection.
      const remoteDevicePeerId = await this.getRemoteDevicePeerId(rtcClient);
      this.assertNotCancelled(cancelled);

      let sessionId: string | undefined = undefined;

      // We can connect our real-time communication client to device peers by their ID
      for (let i = 0; i < maxConnectRetries; i++) {
        sessionId = await rtcClient.connect(remoteDevicePeerId);
        if (!!sessionId) break;
        delay(100);
        this.assertNotCancelled(cancelled);
      }

      if (!sessionId)
        throw new Error(
          `Session could not be created: exhausted ${maxConnectRetries} retries`
        );

      // Wait for the signaling process to complete...
      let retries = 0;
      while (!cancelled) {
        const connectionCompleted =
          rtcClient.getConnectionStatus(remoteDevicePeerId) === "connected";
        if (connectionCompleted) {
          break;
        }

        await delay(100);
        retries += 1;
      }
      this.assertNotCancelled(cancelled);

      console.debug(
        `${new Date().toISOString()} :: Connection completed after ${retries} retries`
      );

      return remoteDevicePeerId;
    };

    return Promise.race([establishConnection(), deadlinePromise])
      .then((remoteDevicePeerId) => {
        this.remoteDevicePeerId = remoteDevicePeerId;
        this.initConnectionMonitoring();
        this.rtcClient = rtcClient;
        this.emit("connect");
      })
      .catch((err) => {
        console.debug(
          `${new Date().toISOString()} :: Connection failed: %o`,
          err
        );
        // cleanup on failure
        this.remoteDevicePeerId = null;
        rtcClient.shutdown().catch((shutdownErr: unknown) => {
          console.error("rtcClient cannot shutdown: %o", shutdownErr);
        });
        this.emit("connection_failed", err);
        throw err;
      });
  }

  private async getRemoteDevicePeerId(rtcClient: RtcClient) {
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

  private initConnectionMonitoring() {
    this.connectionMonitorInterval = setInterval(async () => {
      let dataChannelClosed = false;

      if (this.rtcClient) {
        const rtcConnections = this.rtcClient.getConnections();
        const connection = rtcConnections.find(
          (_) => _.getRemotePeerId() === this.remoteDevicePeerId && _.isActive()
        );
        if (connection === undefined || !connection.isReady()) {
          console.debug(`${new Date().toISOString()} :: data channel closed`);
          dataChannelClosed = true;
        }
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

  async getRemotePeer(): Promise<IRtcPeer> {
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
    let throwNotStartedError = false;

    if (this.rtcClient) {
      this.stopConnectionMonitoring();

      if (this.remoteDevicePeerId) {
        await this.rtcClient.disconnect(this.remoteDevicePeerId);
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

  async isInRealtimeSession(): Promise<boolean> {
    const peers = await getPeers();
    const sessions = await getRealtimeSessions();
    const peer = peers.find((_) => _.deviceId === this.id);
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
      tags: i.tags,
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
    return await getTelemetry(
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
    return await getAnnotationCount({ ...query, deviceIds: [this.id] }, tagKey);
  }

  async getAnnotationCountByIntervals(
    query: IEventQuery,
    tagKey: string,
    aggregate: AggregateLevel
  ) {
    return await getAnnotationCountByIntervals(
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
    return await eventsCounter(eventTypes, timeFrame, range, time, {
      ...query,
      deviceIds: [this.id],
    });
  }

  async createShareLink(share: IShare, view: string) {
    share.scope.deviceIds = [this.id];
    return await createShareLink(share, view);
  }
}
