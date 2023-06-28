export interface TelemetryResult {
  deviceId: string;
  name: string;
  points: [number, any][];
  tags: { [key in string]: string | number };
  type: string;
}
