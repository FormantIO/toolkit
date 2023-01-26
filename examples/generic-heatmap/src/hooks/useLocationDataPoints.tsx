import {
  Authentication,
  Fleet,
  Device,
  IStreamData,
  ILocation,
  IEvent,
  IDataPoint,
} from "@formant/data-sdk";
import { useEffect, useState } from "react";
import { IConfiguration, ILocationAndNumericDataPoint } from "../types";
import { useScrubberTime, useFormant, useDevice } from "@formant/ui-sdk";

const MINUTES = 60000;
const HOURS = MINUTES * 60;

const getEvents = async (
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

const getDataPoints = async (
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

const handleNumericStream = (
  streams: IStreamData<"location" | "numeric">[]
): ILocationAndNumericDataPoint[] => {
  const streamsMap = streams.reduce<{
    // locationStream: IDataPoint<"localization">;
    // numericStream: IDataPoint<"numeric">;
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

  const numericAndLocationMerge: ILocationAndNumericDataPoint[] = [];
  let numericIndex = 0;
  let previusDifference = 0;

  streamsMap.locationStream.forEach(
    (locationDataPoint: IDataPoint<"location">, idx: number) => {
      const numericDataPointTimeStamp =
        streamsMap.numericStream[numericIndex][0];
      const locationDataPointtimeStamp = locationDataPoint[0];

      //GET difference between numeric time stamp and location time stamp
      const currentDifference =
        numericDataPointTimeStamp - locationDataPointtimeStamp;

      //if current difference is negative compare absolute value of current
      // and previus difference
      if (currentDifference < 0) {
        //TODO: HANDLE when there's no location data before the first numeriv value
        Math.abs(currentDifference) < previusDifference
          ? numericAndLocationMerge.push({
              location: locationDataPoint[1],
              weight: streamsMap.numericStream[numericIndex][1],
            })
          : numericAndLocationMerge.push({
              location: streamsMap.locationStream[idx - 1][1],
              weight: streamsMap.numericStream[numericIndex][1],
            });
        numericIndex += 1;
        previusDifference = 0;
      }
      //If the current diference is smaller than the previus diferences
      // update the value of the difference holder
      if (currentDifference < previusDifference) {
        previusDifference = currentDifference;
      }
    }
  );
  return numericAndLocationMerge;
};

const isoDateToMilliseconds = (d: string) => new Date(d).getTime();
const millisecondsToISODate = (m: number) => new Date(m).toISOString();

interface IQuery {
  start: string | number;
  end: string | number;
}

const handleConfiguration = async (
  config: IConfiguration,
  deviceId: string,
  time?: number
) => {
  let query: IQuery = {
    start: "",
    end: "",
  };
  const { start, end } = config;

  switch (start.type) {
    case "Event":
      const events = await getEvents(start.value, deviceId);
      if (events.length < 1) console.error("No events");
      const startTime = isoDateToMilliseconds(events[0].createdAt!);
      query.start = startTime;
      break;
    case "timeRange":
      query.start = time! - HOURS * parseInt(start.value);
      break;
    default:
  }
  switch (end.type) {
    case "Annotation":
      //Start must be name of the event
      let events = await getEvents(start.value, deviceId);
      if (events.length < 1) console.error("No events");
      query.start = events[0].createdAt!;
      query.end = events[0].endTime!;
      break;
    case "Event":
      events = await getEvents(end.value, deviceId);
      if (events.length < 1) console.error("No events");
      query.end = isoDateToMilliseconds(events[0].createdAt!);
      break;
    case "scrubber":
      query.end = time!;
      break;
    case "timeRange":
      query.end = (query.start as number) + HOURS * parseInt(end.value);
      break;
    default:
  }

  if (Number.isNaN(query.start) || !query.end) return null;
  query.start = millisecondsToISODate(query.start as number);
  query.end = millisecondsToISODate(query.end as number);

  return query;
};

export const useLocationDataPoints = ():
  | ILocation[]
  | ILocationAndNumericDataPoint[] => {
  const device = useDevice();
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const time = useScrubberTime();
  const [datapoints, setDatapoints] = useState<
    ILocation[] | ILocationAndNumericDataPoint[]
  >([]);
  const [startTime, setStartTime] = useState<string>();
  const [endTime, setEndTime] = useState<string>();

  useEffect(() => {
    if (!config) return;
    if (config.end.type === "scrubber" || !device) return;
    handleConfiguration(config, device.id).then((_) => {
      setStartTime(_.start as string);
      setEndTime(_.end as string);
    });
  }, [config]);

  useEffect(() => {
    if (!config || config.end.type !== "scrubber" || !device) return;

    handleConfiguration(config, device.id, time).then((_) => {
      if (_ === null) return;
      setStartTime(_.start as string);
      setEndTime(_.end as string);
    });
  }, [config, time, device]);

  useEffect(() => {
    if (!startTime || !endTime || !config) return;
    const { locationStream, numericStream } = config;
    getDataPoints(
      locationStream,
      device.id,
      startTime,
      endTime,
      numericStream
    ).then((_) => {
      if (_ === undefined) {
        console.warn("No streams");
        return;
      }
      if (_.length === 0) {
        setDatapoints([]);
        return;
      }
      if (!!numericStream) {
        const points = handleNumericStream(_);
        setDatapoints(points);
        return;
      }
      setDatapoints(_[0].points.map((d) => d[1]) as ILocation[]);
    });
  }, [device, startTime, endTime]);

  return datapoints;
};
