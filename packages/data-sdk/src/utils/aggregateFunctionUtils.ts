import { INumericAggregate } from "../model/INumericAggregate";
import * as dateFns from "date-fns";
import { AggregateLevel } from "../main";
type IAggregateByDateFunctions = {
  [key in AggregateLevel]: AggregateFunction;
};

const aggregateFunctions = ["interval", "start", "end", "sub", "get"] as const;

export type AggregateFunctions = typeof aggregateFunctions[number];

export type AggregateFunction = {
  [key in AggregateFunctions]: any;
};

export function getVariance(a: INumericAggregate) {
  if (a.count < 2) {
    return 0;
  }
  return a.sumOfSquares / (a.count - 1);
}

export function getStandardDeviation(a: INumericAggregate) {
  return Math.sqrt(getVariance(a));
}

export function getMax(a: INumericAggregate) {
  return a.max;
}

export function getMin(a: INumericAggregate) {
  return a.min;
}

export function getAverage(a: INumericAggregate) {
  return a.count === 0 ? -1 : a.sum / a.count;
}

export function getSum(a: INumericAggregate) {
  return a.sum;
}
export function getCount(a: INumericAggregate) {
  return a.count;
}

export const aggregateFunctionMap = {
  min: getMin,
  max: getMax,
  "standard deviation": getStandardDeviation,
  average: getAverage,
  sum: getSum,
  count: getCount,
};

export const aggregateByDateFunctions: IAggregateByDateFunctions = {
  day: {
    interval: dateFns.eachDayOfInterval,
    start: dateFns.startOfDay,
    end: dateFns.endOfDay,
    sub: dateFns.subDays,
    get: dateFns.getDay,
  },
  week: {
    interval: dateFns.eachWeekOfInterval,
    start: dateFns.startOfWeek,
    end: dateFns.endOfWeek,
    sub: dateFns.subWeeks,
    get: dateFns.getWeek,
  },
  month: {
    interval: dateFns.eachMonthOfInterval,
    start: dateFns.startOfMonth,
    end: dateFns.endOfMonth,
    sub: dateFns.subMonths,
    get: dateFns.getMonth,
  },
  year: {
    interval: dateFns.eachYearOfInterval,
    start: dateFns.startOfYear,
    end: dateFns.endOfYear,
    sub: dateFns.subYears,
    get: dateFns.getYear,
  },
  hour: {
    interval: dateFns.eachHourOfInterval,
    start: dateFns.startOfHour,
    end: dateFns.endOfHour,
    sub: dateFns.subHours,
    get: dateFns.getHours,
  },
  minute: {
    interval: dateFns.eachMinuteOfInterval,
    start: dateFns.startOfMinute,
    end: dateFns.endOfMinute,
    sub: dateFns.subMinutes,
    get: dateFns.getMinutes,
  },
  quarter: {
    interval: dateFns.eachQuarterOfInterval,
    start: dateFns.startOfQuarter,
    end: dateFns.endOfQuarter,
    sub: dateFns.subQuarters,
    get: dateFns.getQuarter,
  },
};

export const formatTimeFrameText = (start: string, end: string) =>
  start.split("/")[0] +
  "/" +
  start.split("/")[1] +
  "–" +
  end.split("/")[0] +
  "/" +
  end.split("/")[1];
