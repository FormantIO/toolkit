export * from "./Fleet";
export * from "./Authentication";
export * from "./Device";
export * from "./PeerDevice";
export * from "./DataChannel";
export * from "./CaptureStream";
export * from "./Manipulator";
export * from "./RequestDataChannel";
export * from "./App";
export * from "./KeyValue";
export * from "./AudioPlayer";

import { Fleet } from "./Fleet";
import { Authentication } from "./Authentication";

let urlParams = new URLSearchParams("");

if (typeof window !== "undefined") {
  urlParams = new URLSearchParams(window.location.search);
}

const urlDevice = urlParams.get("device");
if (urlDevice) {
  Fleet.setDefaultDevice(urlDevice);
}

const urlAuth = urlParams.get("auth");
if (urlAuth) {
  Authentication.loginWithToken(urlAuth);
}

const moduleName = urlParams.get("module");
if (moduleName) {
  Authentication.listenForRefresh();
}

export * from "./model/accessLevels";
export * from "./model/AccessLevel";
export * from "./model/aggregateLevels";
export * from "./model/AggregateLevel";
export * from "./model/AnnotationAreaType";
export * from "./model/annotationTypes";
export * from "./model/AnnotationType";
export * from "./model/EventSortableColumn";
export * from "./model/eventTypes";
export * from "./model/EventType";
export * from "./model/healthStatuses";
export * from "./model/HealthStatus";
export * from "./model/HexRgbColor";
export * from "./model/IAnnotationAreaTypeMap";
export * from "./model/IAnnotation";
export * from "./model/IBaseEntity";
export * from "./model/IBaseEvent";
export * from "./model/IBattery";
export * from "./model/IBitset";
export * from "./model/IBoundingBox";
export * from "./model/ICommandDeliveryEvent";
export * from "./model/ICommandEventBase";
export * from "./model/ICommandParameter";
export * from "./model/ICommandRequestEvent";
export * from "./model/ICommandRequest";
export * from "./model/ICommandResponseEvent";
export * from "./model/ICommandResponse";
export * from "./model/IComment";
export * from "./model/IConfigurationMap";
export * from "./model/ICustomEvent";
export * from "./model/IDataAggregate";
export * from "./model/IDataPoint";
export * from "./model/IEventFilter";
export * from "./model/IEventQuery";
export * from "./model/IEventSort";
export * from "./model/IEvent";
export * from "./model/IFieldParametersTypeMap";
export * from "./model/IFileInfo";
export * from "./model/IFile";
export * from "./model/IFilter";
export * from "./model/IGoalID";
export * from "./model/IGoal";
export * from "./model/IHealth";
export * from "./model/IImageAnnotation";
export * from "./model/IImage";
export * from "./model/IInterventionRequest";
export * from "./model/IInterventionResponse";
export * from "./model/IInterventionTypeMap";
export * from "./model/ILabeledPolygon";
export * from "./model/ILabelingRequestData";
export * from "./model/ILabelingResponseData";
export * from "./model/ILabel";
export * from "./model/ILocalization";
export * from "./model/ILocation";
export * from "./model/IMap";
export * from "./model/interventionTypes";
export * from "./model/InterventionType";
export * from "./model/INumericSetEntry";
export * from "./model/INumericSetAggregateMap";
export * from "./model/INumericAggregate";
export * from "./model/IOdometry";
export * from "./model/IPath";
export * from "./model/IPointCloud";
export * from "./model/IPortForwardingSessionRecord";
export * from "./model/IQuaternion";
export * from "./model/IQuery";
export * from "./model/IRtcSessionRecord";
export * from "./model/ISelectionRequestData";
export * from "./model/ISelectionResponseData";
export * from "./model/ISheetParameters";
export * from "./model/IsoDate";
export * from "./model/ISpreadsheetIdRange";
export * from "./model/IStreamAggregateData";
export * from "./model/IStreamAggregateTypeMap";
export * from "./model/IStreamData";
export * from "./model/IStreamTypeMap";
export * from "./model/IStreamCurrentValue";
export * from "./model/ISystemEvent";
export * from "./model/ITaggedUsers";
export * from "./model/ITagParameters";
export * from "./model/ITagSets";
export * from "./model/ITags";
export * from "./model/ITeleopRequestData";
export * from "./model/ITeleopResponseData";
export * from "./model/ITeleopSessionRecord";
export * from "./model/ITransformNode";
export * from "./model/ITransform";
export * from "./model/ITriggeredEvent";
export * from "./model/ITwist";
export * from "./model/IUserParameters";
export * from "./model/IVector3";
export * from "./model/IVideo";
export * from "./model/severities";
export * from "./model/Severity";
export * from "./model/SortOrder";
export * from "./model/StreamType";
export * from "./model/Timestamp";
export * from "./model/Uuid";
export * from "./model/videoMimeTypes";
export * from "./model/VideoMimeType";
export {
  IRtcSendConfiguration,
  IRtcStreamMessage,
  IRtcStreamPayload,
} from "@formant/realtime-sdk";
