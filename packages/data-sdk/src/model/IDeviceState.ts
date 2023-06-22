import { IDeviceReportedConfigurationState } from "./IDeviceReportedConfigurationState";
import { IHwInfo } from "./IHwInfo";
import { IDeviceRosState } from "./IDeviceRosState";
import { IDictionary } from "./IDictionary";
import { IOnDemandState } from "./IOnDemandState";
import { ICommandProgress } from "./ICommandPropgress";

export interface IDeviceState {
  agentVersion?: string | null;
  reportedConfiguration?: IDeviceReportedConfigurationState | null;
  hwInfo?: IHwInfo | null;
  ros?: IDeviceRosState | null;
  env?: IDictionary | null;
  otaEnabled?: boolean | null;
  onDemand?: IOnDemandState | null;
  commandProgress?: ICommandProgress[] | null;
}
