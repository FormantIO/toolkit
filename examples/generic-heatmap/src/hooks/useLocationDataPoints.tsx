import {
  Authentication,
  Fleet,
  Device,
  IStreamData,
  ILocation,
  IEvent,
} from "@formant/data-sdk";
import { useEffect, useState } from "react";
import { IConfiguration } from "../types";
import { useScrubberTime } from "./useScrubberTime";

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

const getLocationDataPoints = async (
  streamName: string,
  deviceId: string,
  start: string,
  end: string
): Promise<IStreamData<"location">[] | undefined> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const streamsValues = await Fleet.queryTelemetry({
        start,
        end,
        names: [streamName],
        deviceIds: [deviceId],
      });

      return streamsValues as IStreamData<"location">[];
    }
  } catch (error) {
    console.log(error);
  }
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

  query.start = millisecondsToISODate(query.start as number);
  query.end = millisecondsToISODate(query.end as number);

  return query;
};

export const useLocationDataPoints = (
  device: Device,
  config: IConfiguration | null
): ILocation[] => {
  const time = useScrubberTime();
  const [datapoints, setDatapoints] = useState<ILocation[]>([]);
  const [startTime, setStartTime] = useState<string>();
  const [endTime, setEndTime] = useState<string>();

  useEffect(() => {
    if (!config) return;
    if (config.end.type === "scrubber") return;
    handleConfiguration(config, device.id).then((_) => {
      setStartTime(_.start as string);
      setEndTime(_.end as string);
    });
  }, [config]);

  useEffect(() => {
    if (!config) return;
    if (config.end.type !== "scrubber") return;
    handleConfiguration(config, device.id, time?.time).then((_) => {
      setStartTime(_.start as string);
      setEndTime(_.end as string);
    });
  }, [config, time]);

  useEffect(() => {
    if (!startTime || !endTime || !config) return;
    const { locationStream } = config;
    getLocationDataPoints(locationStream, device.id, startTime, endTime).then(
      (_) => {
        if (_ === undefined) {
          console.warn("No streams");
          return;
          //HANDLE NO DATA
        }
        setDatapoints(_[0].points.map((d) => d[1]));
      }
    );
  }, [device, startTime, endTime]);

  return datapoints;
};
