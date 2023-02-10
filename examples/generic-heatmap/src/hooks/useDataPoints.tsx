import { IStreamData, IDataPoint, Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import {
  IHeatMapDataPoint,
  IScrubberConfiguration,
  IAnnotationConfiguration,
  IEventConfiguration,
  ITimeDeltaScrubber,
  IEventScrubber,
  ITimeDeltaEvent,
  IFromEventToEventQuery,
  HeatmapConfiguration,
} from "../types";
import { useScrubberTime, useFormant, useDevice } from "@formant/ui-sdk";
import {
  getEvents,
  getDataPoints,
  generateStreamsMap,
  areDatapointsWithInValidRange,
  updateDifference,
  isoDateToMilliseconds,
  millisecondsToISODate,
  isAnnotationConfiguration,
  isEventConfiguration,
  isScrubberConfiguration,
  isTimeDeltaScrubber,
  isTimeDeltaEvent,
  getTypedConfiguration,
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

const handleEventConfiguration = async (
  config: IEventConfiguration,
  deviceId: string
) => {
  if (isTimeDeltaEvent(config)) {
    return handleEventTimeDelta(config, deviceId);
  } else {
    return handeFromEventToEvent(config, deviceId);
  }
};

const handeFromEventToEvent = async (
  config: IEventConfiguration,
  deviceId: string
) => {
  const eventQuery = config.eventsQuery as IFromEventToEventQuery;
  const endEvent = eventQuery.endEvent;
  const startEvent = eventQuery.startEvent;
  const events = await Fleet.queryEvents({
    names: [endEvent, startEvent],
    deviceIds: [deviceId],
  });
  if (events.length < 1) console.error("No events");

  const query = [];
  let index = 0;
  while (query.length < 2) {
    if (events[index].message === endEvent && query.length === 0) {
      query.push(events[index].time);
    }
    if (query.length === 1 && events[index].message === startEvent) {
      const end = isoDateToMilliseconds(query[0] as string);
      const start = isoDateToMilliseconds(events[index].time);
      if (start < end) {
        query.push(events[index].time);
      }
    }
    if (index === events.length - 1) {
      query.push("", "");
    }
    index += 1;
  }

  return {
    start: query[1],
    end: query[0],
  };
};

const handleEventTimeDelta = async (
  config: IEventConfiguration,
  deviceId: string
) => {
  const eventQuery = config.eventsQuery as ITimeDeltaEvent;
  const endEvent = eventQuery.endEvent;
  const timeDelta = eventQuery.timeDelta;
  const events = await Fleet.queryEvents({
    names: [endEvent],
    deviceIds: [deviceId],
  });
  if (events.length < 1) console.error("No events");
  const end = events[0].time!;
  const start = isoDateToMilliseconds(end) - HOURS * timeDelta;
  return {
    start: millisecondsToISODate(start),
    end: end,
  };
};

const handleScrubberConfiguration = async (
  config: IScrubberConfiguration,
  time: number,
  deviceId: string
) => {
  if (isTimeDeltaScrubber(config)) {
    return handleScrrubberDeltaTime(config, time);
  } else {
    return handleEventScrubber(config, deviceId, time);
  }
};

const handleScrrubberDeltaTime = (
  config: IScrubberConfiguration,
  time: number
) => {
  const scrubber = config.scrubber as ITimeDeltaScrubber;
  const start = time - HOURS * scrubber.timeDelta;
  return {
    start: millisecondsToISODate(start),
    end: millisecondsToISODate(time),
  };
};

const handleEventScrubber = async (
  config: IScrubberConfiguration,
  deviceId: string,
  time: number
) => {
  const scrubber = config.scrubber as IEventScrubber;
  const events = await getEvents(scrubber.eventName, deviceId);
  if (events.length < 1) console.error("No events");
  return {
    start: events[0].time,
    end: millisecondsToISODate(time),
  };
};

const handleAnnotationConfiguration = async (
  config: IAnnotationConfiguration,
  deviceId: string
) => {
  const annotation = config.annotation;
  const annotations = await Fleet.queryEvents({
    deviceIds: [deviceId],
    eventTypes: ["annotation"],
    names: [annotation],
  });
  return {
    start: annotations[0].time,
    end: annotations[0].endTime,
  };
};

export const useDataPoints = (): IHeatMapDataPoint[] => {
  const device = useDevice();
  const context = useFormant();
  const config = getTypedConfiguration(
    context.configuration as HeatmapConfiguration
  );
  const time = useScrubberTime();
  const [datapoints, setDatapoints] = useState<IHeatMapDataPoint[]>([]);
  const [startTime, setStartTime] = useState<string>();
  const [endTime, setEndTime] = useState<string>();

  useEffect(() => {
    if (!config) return;
    const isAnnotation = isAnnotationConfiguration(config);
    if (!isAnnotation || !device) return;
    handleAnnotationConfiguration(config, device.id).then((_) => {
      if (_ === null) return;
      setStartTime(_.start as string);
      setEndTime(_.end as string);
    });
  }, [config, device]);

  useEffect(() => {
    if (!config) return;
    const isScrubber = isScrubberConfiguration(config);
    if (!isScrubber || !device) return;
    handleScrubberConfiguration(config, time as any, device.id).then((_) => {
      if (_ === null) return;
      setStartTime(_.start);
      setEndTime(_.end);
    });
  }, [config, time, device]);

  useEffect(() => {
    if (!config) return;
    const isEvent = isEventConfiguration(config);
    if (!isEvent || !device) return;
    handleEventConfiguration(config, device.id).then((_) => {
      if (_ === null) return;
      setStartTime(_.start);
      setEndTime(_.end);
    });
  }, [config]);

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
        const points = handleNumericStream(_, config.weightSearchWindow);
        setDatapoints(points);
        return;
      }
      setDatapoints(_[0].points.map((d) => d[1]) as IHeatMapDataPoint[]);
    });
  }, [device, startTime, endTime]);

  return datapoints;
};
