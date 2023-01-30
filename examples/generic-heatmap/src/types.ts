export interface IConfiguration {
  locationStream: string;
  start: {
    type: "timeRange" | "Event";
    value: string;
  };
  end: {
    type: "timeRange" | "Event" | "Annotation" | "scrubber";
    value: string;
  };
  numericStream?: string;
  latitude?: number;
  longitude?: number;
  zoom?: number;
  maxSecondsBetweenDatapoints?: number;
  distinctZoomLevel?: number;
  circleRadius?: number;
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
