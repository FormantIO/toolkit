import { IAnnotation } from "./IAnnotation";
import { ICommandDeliveryEvent } from "./ICommandDeliveryEvent";
import { ICommandRequestEvent } from "./ICommandRequestEvent";
import { ICommandResponseEvent } from "./ICommandResponseEvent";
import { IComment } from "./IComment";
import { ICustomEvent } from "./ICustomEvent";
import { IDatapointEvent } from "./IDatapointEvent";
import { IDeviceOfflineEvent } from "./IDeviceOfflineEvent";
import { IDeviceOnlineEvent } from "./IDeviceOnlineEvent";
import { IInterventionRequest } from "./IInterventionRequest";
import { IPortForwardingSessionRecord } from "./IPortForwardingSessionRecord";
import { IStatefulEvent } from "./IStatefulEvent";
import { ISystemEvent } from "./ISystemEvent";
import { ITaskSummary } from "./ITaskSummary";
import { ITeleopSessionRecord } from "./ITeleopSessionRecord";
import { ITriggeredEvent } from "./ITriggeredEvent";

export type IEvent =
  | ITriggeredEvent<"triggered-event">
  | IDatapointEvent
  | IDeviceOnlineEvent
  | IDeviceOfflineEvent
  | IInterventionRequest
  | ITeleopSessionRecord
  | IPortForwardingSessionRecord
  | ICommandRequestEvent
  | ICommandResponseEvent
  | ICommandDeliveryEvent
  | ICustomEvent
  | IComment
  | ISystemEvent
  | IAnnotation
  | ITaskSummary
  | IStatefulEvent;
