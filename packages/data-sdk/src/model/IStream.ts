import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";
import { StreamType } from "./StreamType";

export interface IStream {
  active: boolean;
  alias: string;
  createdAt: IsoDate;
  enabled: boolean;
  id: Uuid;
  isEventFilter: boolean;
  isOverviewColumn: boolean;
  isOverviewRow: boolean;
  isTelemetryFilter: boolean;
  organizationId: Uuid;
  streamName: string;
  streamType: StreamType;
  updatedAt: IsoDate;
}
