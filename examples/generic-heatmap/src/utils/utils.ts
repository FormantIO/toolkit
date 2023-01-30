import { FeatureCollection, Point } from "geojson";
import { IHeatMapDataPoint } from "../types";
import { IEvent, Fleet, Authentication, IStreamData } from "@formant/data-sdk";

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;
const HOURS = MINUTES * 60;

export const generateFeaturesObject = (
  heatMapDataPoints: IHeatMapDataPoint[]
): FeatureCollection<Point> => ({
  type: "FeatureCollection",
  features: heatMapDataPoints.map((d) => ({
    type: "Feature",
    properties: {
      weight: d.weight ?? 50,
    },
    geometry: {
      type: "Point",
      coordinates: [d.longitude, d.latitude],
    },
  })),
});

export const getEvents = async (
  eventName: string,
  deviceId: string
): Promise<IEvent[]> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const events = await Fleet.queryEvents({
        names: [eventName],
        deviceIds: [deviceId],
      });
      return events;
    }
    return [] as IEvent[];
  } catch (error) {
    throw error;
  }
};

export const getDataPoints = async (
  locationStream: string,
  deviceId: string,
  start: string,
  end: string,
  numericStream?: string
): Promise<
  IStreamData<"location" | "numeric">[] | IStreamData<"location">[] | undefined
> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const streamsValues = await Fleet.queryTelemetry({
        start,
        end,
        names: [locationStream, numericStream ?? ""],
        deviceIds: [deviceId],
      });

      return streamsValues as IStreamData<"location" | "numeric">[];
    }
  } catch (error) {
    console.log(error);
  }
};

export const generateStreamsMap = (
  streams: IStreamData<"location" | "numeric">[]
) =>
  streams.reduce<{
    locationStream: any;
    numericStream: any;
  }>(
    (prev, current) => {
      if (current.type === "location") {
        prev.locationStream = current.points;
        return prev;
      }
      prev.numericStream = current.points;
      return prev;
    },
    { locationStream: [], numericStream: [] }
  );

export const areDatapointsWithInValidRange = (
  currentDifference: number,
  maxSecondsBetweenDatapoints: number
) => Math.abs(currentDifference) >= maxSecondsBetweenDatapoints * SECONDS;

export const updateDifference = (currentValue: number, previusValue: number) =>
  currentValue < previusValue ? currentValue : previusValue;

export const isoDateToMilliseconds = (d: string) => new Date(d).getTime();
export const millisecondsToISODate = (m: number) => new Date(m).toISOString();
