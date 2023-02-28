export interface DiagnosticStatusMessage {
  header: {
    frame_id: string;
    seq: number;
    stamp: { secs: number; nsecs: number };
  };
  status: [
    {
      hardware_id: string;
      level: number;
      message: string;
      name: string;
      values: KeyValue[];
    }
  ];
}

export interface KeyValue {
  key: string;
  value: string;
}

export type SeverityLevel = "ok" | "warning" | "critical" | "stale";

export type StatusFilter = {
  [key in SeverityLevel]: boolean;
};

interface data {
  agentId: string;
  deviceId: string;
  name: string;
  tags: { [key: string]: string };
  points: [number, any][];
  type: string;
}

type IStream = [string, Stream];

interface Stream {
  loading: boolean;
  tooMuchData: boolean;
  data: data[];
  type: string;
}

export type Streams = { [key: string]: any };

export interface IConfiguration {
  fullScreenMode: boolean;
  stream: string;
}
