import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { FORMANT_API_URL } from "./config";
import { delay } from "../../common/delay";
import { defined } from "../../common/defined";

export interface ConfigurationDocument {
  urdfFiles: string[];
}

export interface Command {
  id: string;
  name: string;
  command: string;
  description: string;
}

export type RealtimeListener = (peerId: string, message: any) => void;

export type RealtimeVideoStream = {
  name: string;
};
export class Device {
  rtcClient: RtcClient | undefined;
  realtimeListeners: RealtimeListener[] = [];
  constructor(
    private token: string,
    public id: string,
    public name: string,
    private organizationId: string
  ) {}
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
          Authorization: "Bearer " + this.token,
        },
      }
    );
    const telemetry = await data.json();
    return telemetry.items;
  }

  async getCurrentConfiguration(): Promise<ConfigurationDocument> {
    let result = await fetch(`${FORMANT_API_URL}/v1/admin/devices/${this.id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + this.token,
      },
    });
    const device = await result.json();
    const version = device.state.reportedConfiguration.version;
    result = await fetch(
      `${FORMANT_API_URL}/v1/admin/devices/${this.id}/configurations/${version}`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + this.token,
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
        fileId: [fileId],
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + this.token,
      },
    });
    const files = await result.json();
    return files.fileUrls;
  }

  private handleMessage = (peerId: string, message: any) => {
    this.realtimeListeners.forEach((_) => _(peerId, message));
  };

  private getAuthToken = async () => {
    return this.token;
  };

  async startRealtimeConnection() {
    if (!this.rtcClient) {
      const rtcClient = new RtcClient({
        signalingClient: new SignalingPromiseClient(
          FORMANT_API_URL,
          null,
          null
        ),
        getToken: this.getAuthToken,
        receive: this.handleMessage,
      });

      while (!rtcClient.isReady()) {
        await delay(100);
      }

      // Each online device and user has a peer in the system
      const peers = await rtcClient.getPeers();

      // Find the device peer corresponding to the device's ID
      const devicePeer = peers.find((_) => _.deviceId !== undefined);
      if (!devicePeer) {
        // If the device is offline, we won't be able to find its peer.
        console.error("cannot find peer");
        return;
      }

      // We can connect our real-time communication client to device peers by their ID
      const devicePeerId = devicePeer.id;
      await rtcClient.connect(devicePeerId);

      // WebRTC requires a signaling phase when forming a new connection.
      // Wait for the signaling process to complete...
      while (rtcClient.getConnectionStatus(devicePeerId) !== "connected") {
        await delay(100);
      }
      this.rtcClient = rtcClient;
    } else {
      throw new Error(
        `Already created realtime connection to device ${this.id}`
      );
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

  async getRealtimeVideoStreams(): Promise<RealtimeVideoStream[]> {
    const document = (await this.getCurrentConfiguration()) as any;
    const streams = [];
    let videoStream = document.teleop.hardwareStreams[0]?.name as
      | string
      | undefined;
    if (videoStream && videoStream !== "") {
      streams.push({
        name: videoStream,
      });
    }
    videoStream = document.teleop.hardwareStreams[1]?.name as
      | string
      | undefined;
    if (videoStream && videoStream !== "") {
      streams.push({
        name: videoStream,
      });
    }
    videoStream = document.teleop.hardwareStreams[2]?.name as
      | string
      | undefined;
    if (videoStream && videoStream !== "") {
      streams.push({
        name: videoStream,
      });
    }
    return streams;
  }

  async startListeningToRealtimeVideo(stream: RealtimeVideoStream) {
    const client = defined(this.rtcClient);
    const peers = await client.getPeers();

    // Find the device peer corresponding to the device's ID
    const devicePeer = peers.find((_) => _.deviceId !== undefined);
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: true,
      pipeline: "rtc",
    });
  }

  async stopListeningToRealtimeVideo(stream: RealtimeVideoStream) {
    const client = defined(this.rtcClient);
    const peers = await client.getPeers();

    // Find the device peer corresponding to the device's ID
    const devicePeer = peers.find((_) => _.deviceId !== undefined);
    client.controlRemoteStream(defined(devicePeer).id, {
      streamName: stream.name,
      enable: false,
      pipeline: "rtc",
    });
  }

  async stopRealtimeConnection() {
    if (this.rtcClient) {
      await this.rtcClient.disconnect(this.id);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }

  async getAvailableCommands(): Promise<Command[]> {
    const result = await fetch(
      `${FORMANT_API_URL}/v1/admin/command-templates/`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + this.token,
        },
      }
    );
    const commands = await result.json();
    return commands.items.map((i: any) => ({
      name: i.name,
      id: i.id,
      command: i.command,
      description: i.description,
    }));
  }

  async sendCommand(name: string, data: string, time?: Date) {
    const commands = await this.getAvailableCommands();
    const command = commands.find((_) => _.name === name);
    if (!command) {
      throw new Error(`Could not find command with name "${name}"`);
    }
    let parameter = {
      value: data,
      scrubberTime: (time || new Date()).toISOString(),
    };

    const result = await fetch(`${FORMANT_API_URL}/v1/admin/commands`, {
      method: "POST",
      body: JSON.stringify({
        commandTemplateId: command.id,
        organizationId: this.organizationId,
        deviceId: this.id,
        command: command.command,
        parameter: parameter,
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + this.token,
      },
    });
    const files = await result.json();
    return files.fileUrls;
  }
}
