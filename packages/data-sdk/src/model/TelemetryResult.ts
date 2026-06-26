export interface TelemetryResult {
  deviceId: string;
  name: string;
  points: [number, unknown][];
  tags: { [key in string]: string | number };
  type: string;
}
