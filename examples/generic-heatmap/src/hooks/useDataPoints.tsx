import { IStreamData, IDataPoint } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import { IConfiguration, IHeatMapDataPoint, IQuery } from "../types";
import { useScrubberTime, useFormant, useDevice } from "@formant/ui-sdk";
import {
  getEvents,
  getDataPoints,
  generateStreamsMap,
  areDatapointsWithInValidRange,
  updateDifference,
  isoDateToMilliseconds,
  millisecondsToISODate,
} from "../utils/utils";

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;
const HOURS = MINUTES * 60;

const handleNumericStream = (
  streams: IStreamData<"location" | "numeric">[],
  maxSecondsBetweenDatapoints: number = MINUTES * 1
): IHeatMapDataPoint[] => {
  const streamsMap = generateStreamsMap(streams);
  const heatmapDataPoints: IHeatMapDataPoint[] = [];
  let numericIndex = 0;
  let previusDifference = 0;

  streamsMap.locationStream.forEach(
    (locationDataPoint: IDataPoint<"location">, idx: number) => {
      const numericDataPointTimeStamp =
        streamsMap.numericStream[numericIndex][0];
      const locationDataPointTimeStamp = locationDataPoint[0];

      const currentDifference =
        numericDataPointTimeStamp - locationDataPointTimeStamp;

      const validTimeRange = areDatapointsWithInValidRange(
        currentDifference,
        maxSecondsBetweenDatapoints
      );
      if (!validTimeRange) return;

      if (currentDifference < 0) {
        //TODO: HANDLE when there's no location data before the first numeric value
        Math.abs(currentDifference) < previusDifference
          ? heatmapDataPoints.push({
              latitude: locationDataPoint[1].latitude,
              longitude: locationDataPoint[1].longitude,
              weight: streamsMap.numericStream[numericIndex][1],
            })
          : heatmapDataPoints.push({
              latitude: streamsMap.locationStream[idx - 1][1].latitude,
              longitude: streamsMap.locationStream[idx - 1][1].longitude,
              weight: streamsMap.numericStream[numericIndex][1],
            });
        numericIndex += 1;
        if (numericIndex === streamsMap.numericStream.length)
          numericIndex = streamsMap.numericStream.length - 1;
        previusDifference = 0;
      }

      previusDifference = updateDifference(
        currentDifference,
        previusDifference
      );
    }
  );
  return heatmapDataPoints;
};

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

export const useDataPoints = (): IHeatMapDataPoint[] => {
  const device = useDevice();
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const time = useScrubberTime();
  const [datapoints, setDatapoints] = useState<IHeatMapDataPoint[]>([]);
  const [startTime, setStartTime] = useState<string>();
  const [endTime, setEndTime] = useState<string>();

  useEffect(() => {
    if (!config) return;
    if (config.end.type === "scrubber" || !device) return;
    handleConfiguration(config, device.id).then((_) => {
      if (_ === null) return;
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
        const points = handleNumericStream(
          _,
          config.maxSecondsBetweenDatapoints
        );
        setDatapoints(points);
        return;
      }
      setDatapoints(_[0].points.map((d) => d[1]) as IHeatMapDataPoint[]);
    });
  }, [device, startTime, endTime]);

  return datapoints;
};
