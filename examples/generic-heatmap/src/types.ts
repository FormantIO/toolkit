export interface IConfiguration {
  locationStream: string;
  start: {
    type: "timeRange" | "Event";
    value?: string;
    hours?: number;
  };
  end: {
    type: "timeRange" | "Event" | "Annotation" | "scrubber";
    value?: string;
    hours?: number;
  };
  numericStream?: string;
  latitude?: number;
  longitude?: number;
  zoom?: number;
  maxSecondsBetweenDatapoints?: number;
  distinctZoomLevel?: number;
  circleRadius?: number;
  tooltipLabel?: string;
  mapboxKey?: string;
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
