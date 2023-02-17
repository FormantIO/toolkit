const queryTypes = ["Events", "Annotation", "Active Timeline point"] as const;
const startPoints = ["Time Delta", "Event"] as const;

export type HeatmapConfiguration =
  | IScrubberConfiguration
  | IAnnotationConfiguration
  | IEventConfiguration;

type StartPoint = typeof startPoints[number];
type QueryType = typeof queryTypes[number];

interface IStartFrom<T extends StartPoint> {
  startFrom: T;
}

export interface IConfiguration<T extends QueryType> {
  locationStream: string;
  queryType: T;
  numericStream?: string;
  latitude?: number;
  longitude?: number;
  defaultZoomLevel?: number;
  weightSearchWindow?: number;
  distinctZoomLevel?: number;
  circleRadius?: number;
  tooltipLabel?: string;
  mapboxKey?: string;
  heatmapIntensity?: number;
}
export interface ITimeDeltaScrubber extends IStartFrom<"Time Delta"> {
  hours: number;
  minutes: number;
}

export interface IEventScrubber extends IStartFrom<"Event"> {
  eventName: string;
}

export interface IScrubberConfiguration
  extends IConfiguration<"Active Timeline point"> {
  scrubber: ITimeDeltaScrubber | IEventScrubber;
}

export interface IAnnotationConfiguration extends IConfiguration<"Annotation"> {
  annotation: string;
}

export interface ITimeDeltaEvent {
  startFrom: "Time Delta";
  hours: number;
  minutes: number;
  endEvent: string;
}

export interface IFromEventToEventQuery {
  startFrom: "Event";
  startEvent: string;
  endEvent: string;
}

export interface IEventConfiguration extends IConfiguration<"Events"> {
  eventsQuery: ITimeDeltaEvent | IFromEventToEventQuery;
}

export interface IHeatMapDataPoint {
  latitude: number;
  longitude: number;
  weight?: number;
}

export interface IQuery {
  start: string | number;
  end: string | number;
}
