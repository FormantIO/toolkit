import { defined } from "../../common/defined";
import { Authentication } from "./Authentication";
import { FORMANT_API_URL } from "./config";
import { Device } from "./devices/Device";
import { IAnalyticsModule } from "./model/IAnalyticsModule";
import { IDeviceQuery } from "./model/IDeviceQuery";
import { IEvent } from "./model/IEvent";
import { IQuery } from "./model/IQuery";
import { ISqlQuery } from "./model/ISqlQuery";
import { ISqlResult } from "./model/ISqlResult";
import { IStream } from "./model/IStream";
import { IStreamAggregateData } from "./model/IStreamAggregateData";
import { IStreamColumn } from "./model/IStreamColumn";
import { IStreamCurrentValue } from "./model/IStreamCurrentValue";
import { IStreamData } from "./model/IStreamData";
import { IStreamTypeMap } from "./model/IStreamTypeMap";
import { ITaskReportColumn } from "./model/ITaskReportColumn";
import { IView } from "./model/IView";
import { PeerDevice } from "./devices/PeerDevice";
import { IFleet } from "./model/IFleet";
import { createShareLink } from "./api/createShareLink";
import { eventsCounter } from "./api/eventsCounter";
import { getAnnotationCount } from "./api/getAnnotationCount";
import { getAnnotationCountByIntervals } from "./api/getAnnotationCountByIntervals";
import { getTelemetry } from "./api/getTelemetry";
import { getViews } from "./api/getViews";
import { queryEvents } from "./api/queryEvents";
import { getPeers } from "./api/getPeers";
import { getRealtimeSessions } from "./api/getRealtimeSessions";

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
      defined(Authentication.currentOrganization) as string,
      device.tags
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
    const context = new Device(
      deviceId,
      name,
      device.organizationId,
      device.tags
    );
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
        new Device(
          _.id as string,
          _.name as string,
          _.organizationId as string,
          _.tags
        )
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
      (_: any) => new Device(_.id, _.name, _.organizationId, _.tags)
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

  static getPeers = getPeers;

  static getRealtimeSessions = getRealtimeSessions;

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

  static async getLatestTelemetry(
    ...ids: (string | string[])[]
  ): Promise<IStreamCurrentValue<keyof IStreamTypeMap>[]> {
    const deviceIds = ids.flat().filter((_) => !!_);

    if (deviceIds.length === 0) {
      return [];
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

  static getTelemetry = getTelemetry;

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

  static queryEvents = queryEvents;

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

    if (typeof window !== "undefined" && window.location) {
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

  static getAnnotationCount = getAnnotationCount;

  static getAnnotationCountByIntervals = getAnnotationCountByIntervals;

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

  static getViews = getViews;

  static async patchView(view: IView): Promise<IView> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/views/${view.id}`,
      {
        method: "PATCH",
        body: JSON.stringify(view),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await response.json()) as IView;
  }

  static eventsCounter = eventsCounter;

  static async getAnalyticsModules() {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }

    const response = await fetch(
      `${FORMANT_API_URL}/v1/admin/analytics-modules`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await response.json()).items as IAnalyticsModule;
  }

  /**
   * retrieves a list of all available data streams that can be used for running analytics. 
   * This function takes no arguments and returns a list of stream names that can be used for analyzing data. 
   * @example
   * // Returns
   *  [
   *    { 
   *      streamName:  "$.agent.health",
          streamType :  "health"
        },
        { 
   *      streamName:  "up.hours",
          streamType :  "numeric"
        }
      ]
   */

  static async getAnalyticStreams() {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }

    const response = await fetch(
      `${FORMANT_API_URL}/v1/queries/analytics/streams`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await response.json()).items as IStreamColumn[];
  }

  /**
   * retrieves a list of all available tables  that can be used to create task reports.
   * This function takes no arguments and returns a list of table names that can be used for creating task reports.
   * @returns List all available tables
   * @example
   * // Returns
   *[
   *    {
   *       name: "",
   *       tableName: "TASK_REPORTS_CLEANING_MODE",
   *       columns: [
   *                 {
   *                    name: "TYPE",
   *                    isNullable: true,
   *                    dataType: "string",
   *                    tableName: "custom"
   *                 }
   *                ]
   *    }
   *]
   */

  static async getTaskReportTables() {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }

    const response = await fetch(
      `${FORMANT_API_URL}/v1/queries/analytics/task-reports`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return (await response.json()).items as ITaskReportColumn[];
  }

  /**
   *Retrieves all stream rows
   * @example
   * // Body
   * const analytics = await Fleet.queryAnalytics({
   *     aggregateLevel: "day",
   *     orderByColumn: "TIMESTAMP",
   *     streamColumns: [
   *       {
   *         streamName: "consumables_residual_percentage",
   *         streamType: "numeric set",
   *       },
   *     ],
   *   });
   * //Returns
   * {
   *    aggregates: [],
   *    columns: [
   *              {
   *                name: 'TIMESTAMP',
   *                isNullable: true,
   *                dataType: 'string',
   *                tableName: 'NUMERIC_SET_MAIN'
   *               }
   * ],
   *    items: [
   *              {
   *                axisLabel: "suction_blade",
   *                name: "consumables_residual_percentage",
   *                tableName: "NUMERIC_SET_TEST",
   *                time: "2020-04-20T08:00:00.000Z",
   *                type: "numeric set",
   *                unitLabel: "percent"
   *                }
   * ],
   *    rowCount: 14,
   *    rows: []
   *    sqlText: "SELECT dateadd(day, dayofweek(TIMESTAMP), to_timestamp_tz('4/20/2020')) AS TIMESTAMP, SUM(VALUE)"
   * }
   */

  static async queryAnalytics(query: ISqlQuery) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(`${FORMANT_API_URL}/v1/queries/analytics`, {
      method: "POST",
      body: JSON.stringify(query),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return (await response.json()) as ISqlResult;
  }

  /**
   * Retrieves all rows
   * sqlQuery is required
   * @param query
   * @returns
   */

  static async getAnalyticsRows(query: ISqlQuery) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(
      `${FORMANT_API_URL}/v1/queries/analytics/rows`,
      {
        method: "POST",
        body: JSON.stringify(query),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return await response.json();
  }

  /**
   * @param taskColumns is required
   * @returns
   * All task reports
   * @example
   * // Body
   * const tasks = await Fleet.getTaskReports({
   *     taskColumns: [
   *       {
   *         columns: [
   *           {
   *             dataType: "string",
   *             isNullable: true,
   *             name: "TYPE",
   *             tableName: "custom",
   *           },
   *         ],
   *         name: "DURATION_SECONDS",
   *         tableName: "TASK_REPORTS_CLEANING_MODE",
   *         yAxis: "DURATION_SECONDS",
   *       },
   *     ],
   *   });
   */

  static async getTaskReportRows(query: ISqlQuery) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const response = await fetch(
      `${FORMANT_API_URL}/v1/queries/analytics/task-report-rows`,
      {
        method: "POST",
        body: JSON.stringify(query),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return await response.json();
  }

  /**
   * @param scope is required
   * @param time is required
   * @param view View name
   * @returns
   * Share link
   * @example
   * // Body
   * const link = await Fleet.createShareLink({
   *     delegateTeleop: false
   *     message: "See bot in action",
   *     scope: {
   *       deviceIds: ["d64520a6-a308-4a59-9267-b7f8a7bfc7ab"],
   *       start: "2023-04-04T19:51:47.125Z",
   *       end: "2023-04-04T20:51:47.125Z"
   *      },
   *      time: "2023-04-04T20:21:47.125Z",
   *      userName: "User",
   *   });
   */
  static createShareLink = createShareLink;

  static async listFleets(): Promise<IFleet[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const fleets = await data.json();
    return fleets.items;
  }

  static async createFleet(fleet: IFleet): Promise<IFleet> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets`, {
      method: "POST",
      body: JSON.stringify(fleet),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async getFleet(id: string): Promise<IFleet> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async patchFleet(id: string, role: Partial<IFleet>): Promise<IFleet> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}`, {
      method: "PATCH",
      body: JSON.stringify(role),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async deleteFleet(id: string): Promise<void> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    await fetch(`${FORMANT_API_URL}/v1/admin/fleets/${id}`, {
      method: "DELETE",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
  }

  static async addDeviceToFleet(deviceId: string, fleetId: string) {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${deviceId}`,
      {
        method: "PATCH",
        body: JSON.stringify({ fleetId }),
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return await data.json();
  }

  static async getFleetDevices(id: string): Promise<void> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/fleets/${id}/devices`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    const fleets = await data.json();
    return fleets.items;
  }
}
