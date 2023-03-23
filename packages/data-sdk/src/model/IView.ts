import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";

export interface IView {
  id: Uuid;
  createdAt: IsoDate;
  updatedAt: IsoDate;
  organizationId: Uuid;
  name: string;
  url: string;
  showOnSingleDevice: boolean;
  showOnMultiDevice: boolean;
  showOnTeleop: boolean;
  showOnAnalytics: boolean;
  showTimeline: boolean;
}
