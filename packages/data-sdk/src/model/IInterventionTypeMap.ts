// NOTE: Also add an entry in interventionTypes
import { ILabelingRequestData } from "./ILabelingRequestData";
import { ILabelingResponseData } from "./ILabelingResponseData";
import { IPhysicalRequestData } from "./IPhysicalRequestData";
import { IPhysicalResponseData } from "./IPhysicalResponsetData";
import { ISelectionRequestData } from "./ISelectionRequestData";
import { ISelectionResponseData } from "./ISelectionResponseData";
import { ITeleopRequestData } from "./ITeleopRequestData";
import { ITeleopResponseData } from "./ITeleopResponseData";

export interface IInterventionTypeMap {
  selection: {
    request: ISelectionRequestData;
    response: ISelectionResponseData;
  };
  labeling: {
    request: ILabelingRequestData;
    response: ILabelingResponseData;
  };
  teleop: {
    request: ITeleopRequestData;
    response: ITeleopResponseData;
  };
  physical: {
    request: IPhysicalRequestData;
    response: IPhysicalResponseData;
  };
}
