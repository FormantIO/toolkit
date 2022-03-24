import { IAnnotation } from "./IAnnotation";
import { ICommandDeliveryEvent } from "./ICommandDeliveryEvent";
import { ICommandRequestEvent } from "./ICommandRequestEvent";
import { ICommandResponseEvent } from "./ICommandResponseEvent";
import { IComment } from "./IComment";
import { ICustomEvent } from "./ICustomEvent";
import { IInterventionRequest } from "./IInterventionRequest";
import { IPortForwardingSessionRecord } from "./IPortForwardingSessionRecord";
import { ISystemEvent } from "./ISystemEvent";
import { ITeleopSessionRecord } from "./ITeleopSessionRecord";
import { ITriggeredEvent } from "./ITriggeredEvent";

export type IEvent =
  | ITriggeredEvent
  | IInterventionRequest
  | ITeleopSessionRecord
  | IPortForwardingSessionRecord
  | ICommandRequestEvent
  | ICommandResponseEvent
  | ICommandDeliveryEvent
  | ICustomEvent
  | IComment
  | ISystemEvent
  | IAnnotation;
