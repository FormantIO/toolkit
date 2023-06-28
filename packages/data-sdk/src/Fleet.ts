import { defined } from "../../common/defined";
import { Authentication } from "./Authentication";
import { FORMANT_API_URL } from "./config";
import { Device } from "./devices/Device";
import { PeerDevice } from "./devices/PeerDevice";
import { addDeviceToFleet } from "./api/addDeviceToFleet";
import { aggregateTelemetry } from "./api/aggregateTelemetry";
import { createShareLink } from "./api/createShareLink";
import { deleteFleet } from "./api/deleteFleet";
import { eventsCounter } from "./api/eventsCounter";
import { getAnalyticStreams } from "./api/getAnalyticsStreams";
import { getAnalyticsModules } from "./api/getAnalyticsModules";
import { getAnalyticsRows } from "./api/getAnalyticsRows";
import { getAnnotationCount } from "./api/getAnnotationCount";
import { getAnnotationCountByIntervals } from "./api/getAnnotationCountByIntervals";
import { getCurrentGroup } from "./api/getCurrentGroup";
import { getDevice } from "./api/getDevice";
import { getDevices } from "./api/getDevices";
import { getEvent } from "./api/getEvent";
import { getFileUrl } from "./api/getFileUrl";
import { getFleet } from "./api/getFleet";
import { getFleetDevices } from "./api/getFleetDevices";
import { getInterventions } from "./api/getInterventions";
import { getLatestTelemetry } from "./api/getLatestTelemetry";
import { getOnlineDevices } from "./api/getOnlineDevices";
import { getPeers } from "./api/getPeers";
import { getRealtimeDevices } from "./api/getRealtimeDevices";
import { getRealtimeSessions } from "./api/getRealtimeSessions";
import { getStreams } from "./api/getStreams";
import { getTaskReportRows } from "./api/getTaskReportRows";
import { getTaskReportTables } from "./api/getTaskreportTables";
import { getTelemetry } from "./api/getTelemetry";
import { getViews } from "./api/getViews";
import { listFleets } from "./api/listFleets";
import { patchFleet } from "./api/patchFleet";
import { patchStream } from "./api/patchStreams";
import { patchView } from "./api/patchView";
import { queryAnalytics } from "./api/queryAnalytics";
import { queryDevices } from "./api/queryDevices";
import { queryEvents } from "./api/queryEvents";
import { queryTelemetry } from "./api/queryTelemetry";

export class Fleet {
  static defaultDeviceId: string | undefined;
  static knownContext: WeakRef<Device>[] = [];

  static listFleets = listFleets;
  static getFleet = getFleet;
  static patchFleet = patchFleet;
  static deleteFleet = deleteFleet;
  static addDeviceToFleet = addDeviceToFleet;
  static getFleetDevices = getFleetDevices;

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
    const context = await getDevice(deviceId);
    Fleet.knownContext.push(new WeakRef(context));
    return context;
  }

  static aggregateTelemetry = aggregateTelemetry;
  static createShareLink = createShareLink;
  static eventsCounter = eventsCounter;
  static getAnalyticStreams = getAnalyticStreams;
  static getAnalyticsModules = getAnalyticsModules;
  static getAnalyticsRows = getAnalyticsRows;
  static getAnnotationCount = getAnnotationCount;
  static getAnnotationCountByIntervals = getAnnotationCountByIntervals;
  static getCurrentGroup = getCurrentGroup;
  static getDevices = getDevices;
  static getEvent = getEvent;
  static getFileUrl = getFileUrl;
  static getInterventions = getInterventions;
  static getLatestTelemetry = getLatestTelemetry;
  static getOnlineDevices = getOnlineDevices;
  static getPeers = getPeers;
  static getRealtimeDevices = getRealtimeDevices;
  static getRealtimeSessions = getRealtimeSessions;
  static getStreams = getStreams;
  static getTaskReportRows = getTaskReportRows;
  static getTaskReportTables = getTaskReportTables;
  static getTelemetry = getTelemetry;
  static getViews = getViews;
  static patchStream = patchStream;
  static patchView = patchView;
  static queryAnalytics = queryAnalytics;
  static queryDevices = queryDevices;
  static queryEvents = queryEvents;
  static queryTelemetry = queryTelemetry;
}
