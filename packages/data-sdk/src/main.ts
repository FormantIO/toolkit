import { Buffer } from "buffer";
window.Buffer = Buffer;

export { App } from "./App";
export { Fleet } from "./Fleet";
export { Authentication } from "./Authentication";
export { Device } from "./Device";
export { PeerDevice } from "./PeerDevice";
export { DataChannel } from "./DataChannel";
export { CaptureStream } from "./CaptureStream";
export { Manipulator } from "./Manipulator";
export {
  BinaryRequestDataChannel,
  TextRequestDataChannel,
} from "./RequestDataChannel";
export { KeyValue } from "./KeyValue";
export { AudioPlayer } from "./AudioPlayer";
export { Account } from "./Account";
export { Role } from "./Role";
export { User } from "./User";

// Re-exporting core types
export type {
  AppMessage,
  DataPoint,
  EmbeddedAppMessage,
  IDevice,
  ModuleConfigurationMessage,
  ModuleData,
  QueryRange,
  Stream,
  StreamData,
} from "./App";
export type { TelemetryResult } from "./Fleet";
export type {
  Command,
  ConfigurationDocument,
  IAdapterConfiguration,
  IJointState,
  IStartRealtimeConnectionOptions,
  RealtimeAudioStream,
  RealtimeDataStream,
  RealtimeListener,
  RealtimeMessage,
  RealtimeVideoStream,
  TelemetryStream,
} from "./BaseDevice";
export type {
  DataChannelBinaryListener,
  DataChannelErrorListener,
  DataChannelListener,
  DataChannelStringListener,
} from "./DataChannel";
export type { CaptureSession } from "./CaptureStream";
export type { RealtimeManipulatorConfig } from "./Manipulator";

// Application Initialization
import { App } from "./App";
import { Fleet } from "./Fleet";
import { Authentication } from "./Authentication";

try {
  const urlParams =
    typeof window !== "undefined" && window.location
      ? new URLSearchParams(window.location.search)
      : new URLSearchParams("");

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

  if (typeof window !== "undefined") {
    App.listenForConnectionEvents();
  }
} catch (_) {}

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
export * from "./model/IColorRGBA";
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
export * from "./model/IDeviceQuery";
export * from "./model/IAnnotationQuery";
export * from "./model/IStream";
export { SessionTypeConstants as SessionType } from "./model/SessionType";
export type {
  IRtcSendConfiguration,
  IRtcStreamMessage,
  IRtcStreamPayload,
} from "@formant/realtime-sdk";
export * from "./model/JsonSchema";
export * from "./utils/aggregateFunctionUtils";
export * from "./model/IStreamColumn";
export * from "./model/IAnalyticsModule";
export * from "./model/ITaskReportColumn";
export * from "./model/IView";
export * from "./model/IShare";
export * from "./model/ISqlQuery";
export * from "./model/IScopeFilter";
export * from "./model/ISqlColumn";
export * from "./model/ISqlResult";
export * from "./model/IAnalyticsModuleConfiguration";
export * from "./model/IAggregateRow";
export * from "./model/ISqlRow";
export * from "./stores/IAuthenticationStore";
