import { DataSdk } from "./DataSdk";
import { ITags } from "./model/ITags";

export interface CaptureSession {
  deviceId: string;
  streamName: string;
  tags: ITags;
  expiration: string;
  organizationId: string;
  userId: string;
  code: string;
  id: string;
  createdAt: string;
  updatedAt: string;
}

export class CaptureStream {
  token: string | undefined;
  constructor(public captureSession: CaptureSession) {}

  async ingestJSON(value: Record<string, unknown>) {
    if (!this.token) {
      const result = await fetch(
        `${DataSdk.adminApi}/capture-sessions/${this.captureSession.code}/authenticate`,
        {
          method: "POST",
        }
      );
      const authInfo = await result.json();
      this.token = authInfo.token;
    }

    await fetch(DataSdk.ingestionApi, {
      method: "POST",
      body: JSON.stringify({
        deviceId: this.captureSession.deviceId,
        name: this.captureSession.streamName,
        type: "json",
        points: [[Date.now(), JSON.stringify(value)]],
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + this.token,
      },
    });
  }
}
