import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";
import { defined } from "../../common/defined";
import { Authentication } from "./Authentication";
import { FORMANT_API_URL } from "./config";
import { Device } from "./Device";
import { IEvent } from "./model/IEvent";
import { IEventQuery } from "./model/IEventQuery";
import { IQuery } from "./model/IQuery";
import { IStreamAggregateData } from "./model/IStreamAggregateData";
import { IStreamData } from "./model/IStreamData";
import { PeerDevice } from "./PeerDevice";
import { IDeviceQuery } from "./model/IDeviceQuery";
import { IStream } from "./model/IStream";
import { AggregateLevel } from "./main";
import { aggregateByDateFunctions, formatTimeFrameText } from "./main";
import { EventType } from "./main";

export interface TelemetryResult {
  deviceId: string;
  name: string;
  points: [number, any][];
  tags: { [key in string]: string | number };
  type: string;
}

export class Fleet {
  static defaultDeviceId: string | undefined;
  static knownContext: WeakRef<Device>[] = [];

  static async setDefaultDevice(deviceId: string) {
    Fleet.defaultDeviceId = deviceId;
  }

  static async getCurrentDevice(): Promise<Device> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    if (!Fleet.defaultDeviceId) {
      throw new Error("No known default device");
    }

    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/device-details/query`,
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const devices = await data.json();
    const device = devices.items.find(
      (_: { id: string }) => _.id === Fleet.defaultDeviceId
    );
    const name = device.name as string;
    const context = new Device(
      Fleet.defaultDeviceId,
      name,
      defined(Authentication.currentOrganization) as string
    );
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getPeerDevice(url: string): Promise<PeerDevice> {
    const peer = new PeerDevice(url);
    peer.id = await peer.getDeviceId();
    return peer;
  }

  static async getDevice(deviceId: string): Promise<Device> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${deviceId}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const device = await data.json();
    const name = device.name as string;
    const context = new Device(deviceId, name, device.organizationId);
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static async getDevices(): Promise<Device[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/device-details/query`,
      {
        method: "POST",
        body: JSON.stringify({ enabled: true, type: "default" }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const devices = await data.json();
    devices.items;
    return devices.items.map(
      (_: any) =>
        new Device(_.id as string, _.name as string, _.organizationId as string)
    );
  }

  static async queryDevices(query: IDeviceQuery): Promise<Device[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/devices/query`, {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const devices = await data.json();

    return devices.items.map(
      (_: any) => new Device(_.id, _.name, _.organizationId)
    );
  }

  static async getOnlineDevices(): Promise<Device[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/queries/online-devices`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const devices = await data.json();
    const onlineIds = devices.items as string[];
    const allDevices = await Fleet.getDevices();
    return allDevices.filter((_) => onlineIds.includes(_.id));
  }

  static async getPeers(): Promise<IRtcPeer[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const rtcClient = new RtcClient({
      signalingClient: new SignalingPromiseClient(FORMANT_API_URL, null, null),
      getToken: async () => {
        return defined(
          Authentication.token,
          "Realtime when user isn't authorized"
        );
      },
      receive: () => {},
    });
    return await rtcClient.getPeers();
  }

  static async getRealtimeSessions(): Promise<{ [key in string]: string[] }> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const rtcClient = new RtcClient({
      signalingClient: new SignalingPromiseClient(FORMANT_API_URL, null, null),
      getToken: async () => {
        return defined(
          Authentication.token,
          "Realtime when user isn't authorized"
        );
      },
      receive: () => {},
    });
    return await rtcClient.getSessions();
  }

  static async getRealtimeDevices(): Promise<Device[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/signaling/peers`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const devices = await data.json();
    const onlineIds = devices.items.map(
      (_: { deviceId: string }) => _.deviceId
    ) as string[];
    const allDevices = await Fleet.getDevices();
    return allDevices.filter((_) => onlineIds.includes(_.id));
  }

  static async getLatestTelemetry(deviceIdOrDeviceIds?: string | string[]) {
    let deviceIds = deviceIdOrDeviceIds;
    if (deviceIdOrDeviceIds && !Array.isArray(deviceIdOrDeviceIds)) {
      deviceIdOrDeviceIds = [deviceIdOrDeviceIds];
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/queries/stream-current-value`,
      {
        method: "POST",
        body: JSON.stringify({
          deviceIds,
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

  static async getTelemetry(
    deviceIdOrDeviceIds: string | string[],
    streamNameOrStreamNames: string | string[],
    start: Date,
    end: Date,
    tags?: { [key in string]: string[] }
  ): Promise<TelemetryResult[]> {
    let deviceIds = deviceIdOrDeviceIds;
    if (!Array.isArray(deviceIdOrDeviceIds)) {
      deviceIds = [deviceIdOrDeviceIds];
    }
    let streamNames = streamNameOrStreamNames;
    if (!Array.isArray(streamNameOrStreamNames)) {
      streamNames = [streamNameOrStreamNames];
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/queries/queries`, {
      method: "POST",
      body: JSON.stringify({
        deviceIds,
        end: end.toISOString(),
        names: streamNames,
        start: start.toISOString(),
        tags,
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const telemetry = await data.json();
    return telemetry.items;
  }

  static async getFileUrl(uuid: string) {
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/files/query`, {
      method: "POST",
      body: JSON.stringify({
        fileIds: [uuid],
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const result = await data.json();
    if (result.fileUrls.length === 0) {
      throw new Error("File not found");
    }
    return result.fileUrls[0] as string;
  }

  static async queryTelemetry(query: IQuery) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/queries/queries`, {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });

    return (await data.json()).items as IStreamData[];
  }

  static async aggregateTelemetry(query: IQuery) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/queries/queries`, {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });

    return (await data.json()).aggregates as IStreamAggregateData[];
  }

  static async queryEvents(query: IEventQuery): Promise<IEvent[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/events/query`, {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });

    return (await data.json()).items as IEvent[];
  }

  static async getEvent(uuid: string): Promise<IEvent> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/events/query/id=${uuid}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    return (await data.json()).items as IEvent;
  }
  static async getInterventions(): Promise<IEvent[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const interventions = await fetch(
      `${FORMANT_API_URL}/v1/admin/intervention-requests`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await interventions.json()).items as IEvent[];
  }

  static async getCurrentGroup(): Promise<Device[] | undefined> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    let urlParams = new URLSearchParams("");

    if (typeof window !== "undefined") {
      urlParams = new URLSearchParams(window.location.search);
    }

    const groupId = urlParams.get("group");

    if (groupId === null || groupId.trim() === "") {
      return undefined;
    }
    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/groups/` + groupId,
      {
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );

    const { tagKey, tagValue } = await response.json();

    const devices = await this.queryDevices({
      tags: { [tagKey]: [tagValue] },
      enabled: true,
      type: "default",
    });

    return devices;
  }

  static async getAnnotationCount(
    query: IEventQuery,
    annotationName: string,
    tagKey: string
  ) {
    const annotations = await this.queryEvents({
      ...query,
      message: annotationName,
      eventTypes: ["annotation"],
    });

    const validAnnotations = annotations.filter(
      (_) => !!_.tags && Object.keys(_.tags!).includes(tagKey)
    );
    const annotationCounter = validAnnotations.reduce<{
      [key: string]: number;
    }>((prev, current) => {
      const value = current.tags![tagKey];
      if (value in prev) {
        prev[value] += 1;
        return prev;
      }
      prev[value] = 1;
      return prev;
    }, {});

    return annotationCounter;
  }

  static async getAnnotationCountByIntervals(
    query: IEventQuery,
    annotationName: string,
    tagKey: string,
    aggregate: AggregateLevel
  ) {
    const { end, start } = query;
    const dateFunctions = aggregateByDateFunctions[aggregate];
    const intervals: Date[] = dateFunctions.interval({
      start: new Date(start!),
      end: new Date(end!),
    });

    const annotationsQuery = intervals.map((_, idx) => {
      const startDate = new Date(_).toISOString();
      const endDate =
        idx === intervals.length - 1
          ? new Date(Date.now()).toISOString()
          : new Date(intervals[idx + 1]);
      return this.getAnnotationCount(
        {
          ...query,
          start: startDate,
          end: endDate as string,
        },
        annotationName,
        tagKey
      );
    });
    const responses = await Promise.all(annotationsQuery);

    return intervals.map((_, idx) => ({
      date: new Date(_).toISOString(),
      annotations: responses[idx],
    }));
  }

  static async getStreams(): Promise<IStream[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(`${FORMANT_API_URL}/v1/admin/streams`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const streams = await response.json();
    return streams.items.filter(
      (_: { enabled: boolean }) => _.enabled
    ) as IStream[];
  }

  static async patchStream(stream: IStream): Promise<IStream> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/streams/${stream.id}`,
      {
        method: "PATCH",
        body: JSON.stringify(stream),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await response.json()) as IStream;
  }

  static async eventsCounter(
    eventTypes: EventType[],
    timeFrame: AggregateLevel,
    range: number,
    time: number,
    query?: IEventQuery
  ) {
    const dateFunctions = aggregateByDateFunctions[timeFrame];

    return await Promise.all(
      Array(range)
        .fill(0)
        .map(async (_, dateOffset) => {
          const activePointInTimeLine = new Date(time);

          const startDate: Date = dateFunctions.sub(
            dateFunctions.start(activePointInTimeLine),
            range - dateOffset - 1
          );
          const endDate: Date = dateFunctions.sub(
            dateFunctions.end(activePointInTimeLine),
            range - dateOffset - 1
          );
          const date = formatTimeFrameText(
            startDate.toLocaleDateString(),
            endDate.toLocaleDateString()
          );
          const events = await Fleet.queryEvents({
            ...query,
            eventTypes,
            start: new Date(startDate).toISOString(),
            end: new Date(endDate).toISOString(),
          });
          return { date, events };
        })
    );
  }
}
