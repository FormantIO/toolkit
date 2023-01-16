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
}
