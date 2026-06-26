import { IFilter } from "../model/IFilter";

export interface IView {
  id: string;
  tags: { [key: string]: string };
  organizationId: string;
  name: string;
  description?: string | null;
  url?: string | null;
  showOnSingleDevice?: boolean;
  showOnMultiDevice?: boolean;
  showOnTeleop?: boolean;
  showOnAnalytics?: boolean;
  showTimeline?: boolean;
  localModeEnabled?: boolean;
  filter: IFilter | null;
  deviceFilter: IFilter | null;
  groupFilter: IFilter | null;
  layoutType?: string | null;
  index: number;
}
