import { Authentication, INumericAggregate, Fleet } from "@formant/data-sdk";
import { BarChart, useFormant, LoadingIndicator } from "@formant/ui-sdk";
import * as dateFns from "date-fns";
import { useEffect, useMemo, useState } from "react";
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
} from "./types";
import { getTypedConfiguration } from "./utils/getTypedConfiguration";

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

export function NumericAggregateBar() {
  const context = useFormant();
  const config = getTypedConfiguration(
    context.configuration as ConfigurationTypes
  );
  const [arr, setArr] = useState([0]);
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

  useEffect(() => {
    loadValues();
  }, [
    _numAggregates,
    (config as INumericSetConfiguration).numericSetStream,
    (config as INumericConfiguration).numericStream,
  ]);

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
              aggregate: _aggregateBy,
              deviceIds: deviceIds ?? [currentDevice.id],
              names: [
                streamType === "numeric"
                  ? config.numericStream
                  : config.numericSetStream,
              ],
            }),
          };
        })
      );
      const aggregations = aggregatedData.map((streamDatas) => {
        if (streamDatas.data === undefined) {
          return undefined;
        }
        //TODO: HANLDE TAGGED DATA
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
      (_, i) =>
        `${capitalizeFirstLetter(
          _aggregateBy
        )} ${_.start.getDate()}/${_.start.getMonth()}`
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
        <LoadingIndicator />
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
