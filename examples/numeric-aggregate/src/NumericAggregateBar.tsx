import { Authentication, Fleet, INumericAggregate } from "@formant/data-sdk";
import { BarChart } from "@formant/ui-sdk";
import * as dateFns from "date-fns";
import { useEffect, useMemo, useState } from "react";
import "./App.css";
import { aggregateFunctionMap } from "./utils/aggregateFunctionUtils";
import {
  reduceNumericSetStreamAggregates,
  reduceNumericStreamAggregates,
} from "./utils/numericAggregateUtils";
import { AggregateType, AggregatePeriod } from "./types";

interface INumericAggregateBarProps {
  streamName: string;
  aggregateType: AggregateType;
  numericSetKey?: string;
  aggregateBy?: AggregatePeriod;
  numAggregates?: number;
}

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

export function NumericAggregateBar(props: INumericAggregateBarProps) {
  const [arr, setArr] = useState([0]);
  const [aggregations, setAggregations] = useState<
    { start: Date; aggregate: INumericAggregate }[] | undefined
  >();
  const {
    streamName,
    aggregateType,
    numericSetKey,
    aggregateBy: propsAggregateBy,
    numAggregates: propsNumAggregates,
  } = props;
  const aggregateBy = propsAggregateBy ?? defaultAggregtateBy;
  const numAggregates = propsNumAggregates ?? defaultNumAggregates;
  const dateFunctions = aggregateByDateFunctions[aggregateBy];
  useEffect(() => {
    setArr(new Array(numAggregates).fill(0));
  }, [numAggregates]);

  const something = useMemo(() => {
    const _ = new Array(parseInt(numAggregates)).fill(0);
    console.log(_, numAggregates);
    return _;
  }, [numAggregates]);

  useEffect(() => {
    loadValues();
  }, [numAggregates]);

  const loadValues = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      const aggregatedData = await Promise.all(
        something.map(async (_, dateOffset) => {
          const now = new Date();
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
              aggregate: aggregateBy,
              deviceIds: [currentDevice.id],
              names: [streamName],
            }),
          };
        })
      );

      console.log(aggregatedData);
      const aggregations = aggregatedData.map((streamDatas) => {
        if (streamDatas.data === undefined) {
          return undefined;
        }
        if (streamDatas.data.length > 1) {
          console.log("stream data is long");
          return undefined;
        }
        const { start, data } = streamDatas;
        const stream = data[0];
        if (stream.type == "numeric") {
          return {
            start,
            aggregate: reduceNumericStreamAggregates(stream),
          };
        }
        if (stream.type == "numeric set") {
          if (numericSetKey === undefined) {
            console.log("Is numeric set but no key");
            return undefined;
          }
          return {
            start,
            aggregate: reduceNumericSetStreamAggregates(stream, numericSetKey),
          };
        }
        return undefined;
      });
      const filteredAggregations = aggregations.filter(
        (_): _ is { start: Date; aggregate: INumericAggregate } =>
          !!_?.aggregate
      );
      console.log(filteredAggregations);
      setAggregations(filteredAggregations.reverse());
    }
  };

  const data = aggregations?.map((_) => {
    return aggregateFunctionMap[aggregateType](_.aggregate);
  });
  const xMax = data ? Math.max(...data) : 100;
  const labels =
    aggregations?.map(
      (_, i) =>
        `${capitalizeFirstLetter(
          aggregateBy
        )} ${_.start.getDate()}/${_.start.getMonth()}`
    ) ?? [];

  const aggregateByAdverb = {
    day: "daily",
    week: "weekly",
    month: "monthly",
  };
  const titleString = `${capitalizeFirstLetter(
    aggregateByAdverb[aggregateBy]
  )} ${aggregateType} of ${streamName}${
    numericSetKey ? "." + numericSetKey : ""
  }`;
  return (
    <>
      {data ? (
        <>
          <h3>{titleString}</h3>
          <BarChart
            labels={labels}
            xMax={generateReasonableNearest(xMax)}
            data={data}
            height={250}
            width={400}
          />
        </>
      ) : (
        "Loading..."
      )}
    </>
  );
}

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
