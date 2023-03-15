import { Authentication, INumericAggregate, Fleet } from "@formant/data-sdk";
import {
  BarChart,
  useFormant,
  LoadingIndicator,
  useScrubberTime,
} from "@formant/ui-sdk";
import * as dateFns from "date-fns";
import { FC, useEffect, useMemo, useState } from "react";
import "./App.css";
import { aggregateFunctionMap } from "./utils/aggregateFunctionUtils";
import {
  reduceNumericSetStreamAggregates,
  reduceNumericStreamAggregates,
} from "./utils/numericAggregateUtils";
import {
  ConfigurationTypes,
  INumericConfiguration,
  INumericSetConfiguration,
  ITag,
} from "./types";
import { getTypedConfiguration } from "./utils/getTypedConfiguration";
import { dummyData } from "./utils/dummyData";

const defaultAggregtateBy = "week";
const defaultNumAggregates = 4;

const aggregateByDateFunctions = {
  day: {
    start: dateFns.startOfDay,
    end: dateFns.endOfDay,
    sub: dateFns.subDays,
  },
  week: {
    start: dateFns.startOfWeek,
    end: dateFns.endOfWeek,
    sub: dateFns.subWeeks,
  },
  month: {
    start: dateFns.startOfMonth,
    end: dateFns.endOfMonth,
    sub: dateFns.subMonths,
  },
};

interface INumericAggregateBarProps {
  time: number;
  config: INumericConfiguration | INumericSetConfiguration;
}

const formatTags = (tagArray?: ITag[]) => {
  if (!tagArray) return undefined;
  return tagArray.reduce<any>((p, c) => {
    p[c.key] = [c.value];
    return p;
  }, {});
};

export const NumericAggregateBar: FC<INumericAggregateBarProps> = ({
  time,
  config,
}) => {
  const [height, setHeight] = useState(200);
  const [width, setWidth] = useState(400);
  const [arr, setArr] = useState([0]);

  const handleResize = (e: UIEvent) => {
    const newHeight =
      window.innerHeight * 0.7 < 250 ? 250 : window.innerHeight * 0.7;
    const newWidth =
      window.innerWidth * 0.8 < 300 ? 300 : window.innerWidth * 0.8;

    setHeight(newHeight);
    setWidth(newWidth);
  };
  useEffect(() => {
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
    };
  }, []);

  const [aggregations, setAggregations] = useState<
    { start: Date; aggregate: INumericAggregate }[] | undefined
  >();
  const { aggregateType, aggregateBy, numAggregates, deviceIds, streamType } =
    config;

  const _aggregateBy = aggregateBy ?? defaultAggregtateBy;
  const _numAggregates = numAggregates ?? defaultNumAggregates;
  const dateFunctions = aggregateByDateFunctions[_aggregateBy];

  useEffect(() => {
    setArr(new Array(_numAggregates).fill(0));
  }, [_numAggregates]);

  const something = useMemo(() => {
    const _ = new Array(_numAggregates).fill(0);
    return _;
  }, [_numAggregates]);

  const currentTime = useMemo(() => {
    if (!time || !config) return;

    const date = new Date(config.fullScreenMode ? Date.now() : time);

    const year = date.getUTCFullYear();
    const month = date.getUTCMonth();
    const day = date.getUTCDate();

    return `${month + 1}/${day}/${year}`;
  }, [time]);

  useEffect(() => {
    if (!currentTime) return;
    loadValues();
  }, [
    _numAggregates,
    (config as INumericSetConfiguration).numericSetStream,
    (config as INumericConfiguration).numericStream,
    config.fullScreenMode,
    currentTime,
  ]);

  const loadValues = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      const aggregatedData = await Promise.all(
        something.map(async (_, dateOffset) => {
          const now = new Date(currentTime!);
          const startDate = dateFunctions.sub(
            dateFunctions.start(now),
            dateOffset
          );
          const endDate = dateFunctions.sub(dateFunctions.end(now), dateOffset);
          return {
            start: startDate,
            data: await Fleet.aggregateTelemetry({
              start: startDate.toISOString(),
              end: endDate.toISOString(),
              aggregate: _aggregateBy,
              deviceIds: deviceIds ?? [currentDevice.id],
              names: [
                streamType === "numeric"
                  ? config.numericStream
                  : config.numericSetStream,
              ],
              tags: formatTags(config.tags),
            }),
          };
        })
      );

      const aggregations = aggregatedData.map((streamDatas) => {
        if (streamDatas.data === undefined) {
          return undefined;
        }

        const { start, data } = streamDatas;
        const stream = data[0];
        if (stream.type === "numeric") {
          return {
            start,
            aggregate: reduceNumericStreamAggregates(stream),
          };
        }
        if (stream.type === "numeric set") {
          if (
            (config as INumericSetConfiguration).numericSetKey === undefined
          ) {
            console.log("Is numeric set but no key");
            return undefined;
          }
          return {
            start,
            aggregate: reduceNumericSetStreamAggregates(
              stream,
              (config as INumericSetConfiguration).numericSetKey
            ),
          };
        }
        return undefined;
      });

      const filteredAggregations = aggregations.filter(
        (_): _ is { start: Date; aggregate: INumericAggregate } =>
          !!_?.aggregate
      );
      setAggregations(filteredAggregations.reverse());
    }
  };

  const data = aggregations?.map((_) => {
    return aggregateFunctionMap[aggregateType](_.aggregate);
  });
  const xMax = data ? Math.max(...data) : 100;
  const labels =
    aggregations?.map(
      (_, i) => `${_.start.getMonth() + 1}/${_.start.getDate()}`
    ) ?? [];

  const aggregateByAdverb = {
    day: "daily",
    week: "weekly",
    month: "monthly",
  };
  const titleString = `${capitalizeFirstLetter(
    aggregateByAdverb[_aggregateBy]
  )} ${aggregateType} of ${
    streamType === "numeric" ? config.numericStream : config.numericSetStream
  }${
    (config as INumericSetConfiguration).numericSetKey
      ? "." + (config as INumericSetConfiguration).numericSetKey
      : ""
  }`;
  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        flexDirection: "column",
        justifyContent: "center",
      }}
    >
      {data ? (
        <>
          <h3 style={{ fontSize: height < 251 ? 14 : 18 }}>{titleString}</h3>
          <BarChart
            labels={labels}
            xMax={generateReasonableNearest(xMax)}
            data={data}
            height={height}
            width={width}
          />
        </>
      ) : (
        <LoadingIndicator />
      )}
    </div>
  );
};

function capitalizeFirstLetter(s: string) {
  return s.charAt(0).toUpperCase() + s.slice(1);
}

function generateReasonableNearest(num: number) {
  return num < 10
    ? roundUpNearest(num, 5)
    : num < 75
    ? roundUpNearest(num, 10)
    : num < 100
    ? roundUpNearest(num, 100)
    : num < 250
    ? roundUpNearest(num, 10)
    : num < 2500
    ? roundUpNearest(num, 100)
    : roundUpNearest(num, 1000);
}
function roundUpNearest(num: number, nearest: number) {
  return Math.ceil(num / nearest) * nearest;
}
