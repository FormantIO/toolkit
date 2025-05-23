/* eslint-disable @typescript-eslint/no-unused-vars, @typescript-eslint/no-explicit-any, @typescript-eslint/ban-ts-comment */

import { subDays } from "date-fns";

import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";

import {
  IRtcStreamMessage,
  createRtcStreamMessage,
} from "@formant/realtime-sdk";
import { ITransform } from "@formant/realtime-sdk/dist/model/ITransform";
import { SessionType } from "@formant/realtime-sdk/dist/protos/api/signaling/v1/signaling_pb";
import { Device } from "../../devices/Device";
import { PeerDevice } from "../../devices/PeerDevice";
import { Fleet } from "../../Fleet";
import { IBitset } from "../../model/IBitset";
import { ILocation } from "../../model/ILocation";
import { IQuery } from "../../model/IQuery";
import { IStreamData } from "../../model/IStreamData";
import { ITransformNode } from "../../model/ITransformNode";
// @ts-ignore
import RealtimePlayerWorker from "../../../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

import {
  CloseSubscription,
  DataSourceState,
  IRealtimeStream,
  ITelemetryRosStream,
  ITelemetryStream,
  IUniverseStatistics,
  Interaction,
  RealtimeButtonConfiguration,
  UniverseDataSource,
} from "../model/IUniverseData";
import { DataLoader } from "./loader/DataLoader";
import { QueryStore } from "./queryStore";

export type DeviceId = string;
export type DataSourceId = string;
export type DataResult<T> = {
  deviceId: DeviceId;
  sourceId: DataSourceId;
  data: T | undefined;
};

// get query paramters "debug"
const debug =
  new URLSearchParams(window.location.search).get("debug") === "true";

export class BasicUniverseDataConnector {
  subscriberSources: Map<string, Map<string, UniverseDataSource>> = new Map();

  subscriberLoaders: Map<
    DeviceId,
    Map<DataSourceId, (data: any) => Promise<DataResult<any>>>
  > = new Map();

  subscriberDistributorsLoaders: Map<
    DeviceId,
    Map<DataSourceId, ((data: any) => void)[]>
  > = new Map();

  mapRealtimeConnections: Map<string, Device | PeerDevice | "loading"> =
    new Map();

  lastQueriedHistoricTime: Date | undefined;
  time: Date | "live";

  timeChangeListeners: ((time: Date | "live") => void)[] = [];

  queryStore: QueryStore = new QueryStore();
  dataLoader: any;

  setTime(time: Date | "live"): void {
    if (time !== "live") {
      this.time = time;
    }
    this.timeChangeListeners.forEach((listener) => listener(time));
  }

  getTime(): Date | "live" {
    return this.time;
  }

  getTimeMs(): number {
    return this.time === "live" ? new Date().getTime() : this.time.getTime();
  }

  constructor() {
    this.time = "live";
    this.dataLoader = DataLoader.get();

    const dataLoop = async () => {
      if (Array.from(this.subscriberLoaders.keys()).length > 0) {
        // Load all data for this time
        const deviceIds: string[] = [];
        const query = this.generateTelemetryFilter();
        const data = this.queryStore.query(query);
        if (!data || data === "too much data") {
          return;
        }
        data.forEach((_) => {
          deviceIds.push(_.deviceId);
        });
        const uniqueDeviceIds = Array.from(new Set(deviceIds));
        const loaders: Promise<DataResult<any>>[] = [];
        uniqueDeviceIds.forEach((deviceId) => {
          const subscribers = this.subscriberLoaders.get(deviceId);
          if (subscribers) {
            subscribers.forEach((loader) => {
              loaders.push(loader(data));
            });
          }
        });
        const results = await Promise.all(loaders);
        // distrubute it all at the same time
        results.forEach((result) => {
          if (result.data) {
            const subscribers = this.subscriberDistributorsLoaders.get(
              result.deviceId
            );
            if (subscribers) {
              const subscribersForDeviceAndSource = subscribers.get(
                result.sourceId
              );
              if (subscribersForDeviceAndSource) {
                subscribersForDeviceAndSource.forEach((subscriber) => {
                  subscriber(result.data);
                });
              }
            }
          }
        });
      }
      // start again
      setTimeout(() => dataLoop(), 0);
    };
    setTimeout(() => dataLoop(), 0);
  }

  private generateTelemetryFilter(): IQuery {
    const deviceIds = Array.from(this.subscriberSources.keys());
    const names: string[] = [];
    deviceIds.forEach((deviceId) => {
      const sources = this.subscriberSources.get(deviceId);
      if (sources) {
        Array.from(sources?.values()).forEach((source) => {
          if (source.sourceType === "telemetry") {
            names.push(source.streamName);
          }
        });
      }
    });
    return {
      deviceIds,
      names,
      start: subDays(new Date(), 20).toISOString(),
      end: new Date().toISOString(),
      latestOnly: true,
    };
  }

  protected async sendRtcMessage(deviceId: string, msg: IRtcStreamMessage) {
    await this.createRealtimeConnection(deviceId);
    const device = this.mapRealtimeConnections.get(deviceId);
    if (device && device !== "loading" && device) {
      device.sendRealtimeMessage(msg, {
        channelLabel: "stream.reliable",
      });
    }
  }

  protected async createRealtimeConnection(
    deviceId: string,
    sessionType: number = SessionType.TELEOP
  ) {
    const existingDevice = this.mapRealtimeConnections.get(deviceId);
    if (existingDevice === undefined) {
      this.mapRealtimeConnections.set(deviceId, "loading");

      // is url
      const isUrl = deviceId.startsWith("http");
      console.log("Creating {} realtime connection", isUrl ? "peer" : "cloud");

      let device;
      if (isUrl) {
        device = new PeerDevice(deviceId);
      } else {
        device = await Fleet.getDevice(deviceId);
      }

      await device.startRealtimeConnection(sessionType);
      if (debug) {
        device.addRealtimeListener((peerId, msg) => {
          console.log("received message from peer", peerId, msg);
        });
      }
      this.mapRealtimeConnections.set(deviceId, device);
    } else if (existingDevice === "loading") {
      await new Promise<void>((resolve) => {
        const i = setInterval(() => {
          const device = this.mapRealtimeConnections.get(deviceId);
          if (device && device !== "loading") {
            clearInterval(i);
            resolve();
          }
        }, 100);
      });
    }
  }

  protected addRemovableTelemetrySubscription<T>(
    deviceId: string,
    source: UniverseDataSource,
    loader: (data: IStreamData[]) => Promise<DataResult<T>>,
    distribute: (data: T) => void
  ) {
    let deviceSubscribers = this.subscriberLoaders.get(deviceId);
    if (!deviceSubscribers) {
      deviceSubscribers = new Map();
      this.subscriberLoaders.set(deviceId, deviceSubscribers);
    }
    const subscribers = deviceSubscribers.get(source.id);
    if (!subscribers) {
      deviceSubscribers.set(source.id, loader);
    }
    let distributorsForDevices =
      this.subscriberDistributorsLoaders.get(deviceId);
    if (!distributorsForDevices) {
      distributorsForDevices = new Map();
      this.subscriberDistributorsLoaders.set(deviceId, distributorsForDevices);
    }
    const distributors = distributorsForDevices.get(source.id);
    if (!distributors) {
      distributorsForDevices.set(source.id, [distribute]);
    } else {
      distributors.push(distribute);
    }
    let deviceSubscriberSources = this.subscriberSources.get(deviceId);
    if (!deviceSubscriberSources) {
      deviceSubscriberSources = new Map();
      this.subscriberSources.set(deviceId, deviceSubscriberSources);
    }
    deviceSubscriberSources.set(source.id, source);
    return () => {
      const deviceSubscribersForRemoval = this.subscriberLoaders.get(deviceId);
      if (deviceSubscribersForRemoval) {
        deviceSubscribersForRemoval.delete(source.id);
      }
      const distributorsForRemoval =
        this.subscriberDistributorsLoaders.get(deviceId);
      if (distributorsForRemoval) {
        const dataSourceDistributors = distributorsForRemoval.get(source.id);
        if (dataSourceDistributors) {
          const index = dataSourceDistributors.indexOf(distribute);
          if (index > -1) {
            dataSourceDistributors.splice(index, 1);
          }
        }
      }
      const deviceSubscriberSourcesForRemoval =
        this.subscriberSources.get(deviceId);
      if (deviceSubscriberSourcesForRemoval) {
        deviceSubscriberSourcesForRemoval.delete(source.id);
      }
    };
  }

  protected createH264Drawer(): H264BytestreamCanvasDrawer {
    const drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    return drawer;
  }

  async sendCommand(
    deviceId: string,
    name: string,
    data?: string | undefined
  ): Promise<void> {
    const d = this.mapRealtimeConnections.get(deviceId);
    if (d === "loading" || d === undefined) {
      throw new Error("Device is not ready or doesnt exist");
    }
    await d.sendCommand(name, data);
  }

  async sendRealtimePose(
    deviceId: string,
    streamName: string,
    pose: ITransform
  ): Promise<void> {
    const msg = createRtcStreamMessage(
      {
        entityId: deviceId,
        streamName,
        streamType: "pose",
      },
      {
        pose,
      },
      "vision"
    );
    await this.sendRtcMessage(deviceId, msg);
  }

  async sendRealtimeBoolean(
    deviceId: string,
    streamName: string,
    value: boolean
  ): Promise<void> {
    const msg = createRtcStreamMessage(
      {
        entityId: deviceId,
        streamName,
        streamType: "boolean",
      },
      {
        boolean: value,
      }
    );
    await this.sendRtcMessage(deviceId, msg);
  }

  async sendRealtimeBitset(
    deviceId: string,
    streamName: string,
    bitset: IBitset
  ): Promise<void> {
    const msg = createRtcStreamMessage(
      {
        entityId: deviceId,
        streamName,
        streamType: "bitset",
      },
      {
        bitset: {
          bits: bitset.keys.map((key: string, i: number) => ({
            key,
            value: bitset.values[i],
          })),
        },
      }
    );
    await this.sendRtcMessage(deviceId, msg);
  }

  async getUrdfs(deviceId: string): Promise<string[]> {
    const device = await Fleet.getDevice(deviceId);
    const config = await device.getConfiguration();
    if (!config.urdfFiles || config.urdfFiles.length === 0) {
      return [];
    }
    const zipFileUrl = await Fleet.getFileUrl(config.urdfFiles[0]);
    return [zipFileUrl];
  }

  async getTelemetryStreams(_deviceId: string): Promise<ITelemetryStream[]> {
    throw new Error("Not implemented");
  }

  async getTeleopRosStreams(_deviceId: string): Promise<ITelemetryRosStream[]> {
    throw new Error("Not implemented");
  }

  async getHardwareStreams(_deviceId: string): Promise<IRealtimeStream[]> {
    throw new Error("Not implemented");
  }

  addInteraction(_interaction: Interaction): void {
    throw new Error("Method not implemented.");
  }

  removeInteraction(_id: string): void {
    throw new Error("Method not implemented.");
  }

  getInteractions(): Interaction[] {
    throw new Error("Method not implemented.");
  }

  addInteractionsChangedListener(
    _callback: (interactions: Interaction[]) => void
  ): () => void {
    throw new Error("Method not implemented.");
  }

  addInteractionListener(
    _callback: (interaction: Interaction) => void
  ): () => void {
    throw new Error("Method not implemented.");
  }

  getRealtimeButtons(
    _deviceId: string
  ): Promise<RealtimeButtonConfiguration[]> {
    throw new Error("Method not implemented.");
  }

  async getLatestTransformTrees(
    _deviceId: string
  ): Promise<{ streamName: string; transformTree: ITransformNode }[]> {
    throw new Error("Not implemented");
  }

  async getLatestLocations(): Promise<
    {
      streamName: string;
      location: ILocation;
    }[]
  > {
    return [
      {
        streamName: "spotLocation",
        location: {
          latitude: 45.4661989,
          longitude: -122.5782375,
        },
      },
    ];
  }

  async getDeviceContexts(): Promise<
    { deviceName: string; deviceId: string }[]
  > {
    return [];
  }

  async getDeviceContextName(_deviceId: string): Promise<string | undefined> {
    throw new Error("Not implemented");
  }

  async getTelemetryStreamType(
    _deviceId: string,
    _streamName: string
  ): Promise<string | undefined> {
    throw new Error("Not implemented");
  }

  async getStatistics(): Promise<IUniverseStatistics> {
    return {
      rtcDevices: [],
    };
  }

  subscribeDataSourceStateChange(
    _deviceId: string,
    _source: UniverseDataSource,
    _onDataSourceStateChange?: (state: DataSourceState) => void
  ): CloseSubscription {
    return () => {};
  }

  async fetchImage(url: string): Promise<HTMLImageElement> {
    const image = new Image();
    image.src = url;
    image.setAttribute("crossOrigin", "");
    await new Promise((resolve) => {
      image.onload = resolve;
    });
    return image;
  }
}
