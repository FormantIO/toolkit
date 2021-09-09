import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { FORMANT_API_URL } from "./main";
import { delay } from "./utils";

export interface ConfigurationDocument {
  urdfFiles: string[];
}

export type RealtimeListener = (peerId: string, message: any) => void;
export class Device {
  rtcClient: RtcClient | undefined;
  realtimeListeners: RealtimeListener[] = [];
  constructor(private token: string, public id: string, public name: string) {}
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

  async stopRealtimeConnection() {
    if (this.rtcClient) {
      await this.rtcClient.disconnect(this.id);
    } else {
      throw new Error(`Realtime connection hasn't been started for ${this.id}`);
    }
  }
}
