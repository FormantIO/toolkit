import { INumericAggregate } from "../model/INumericAggregate";
import * as dateFns from "date-fns";
import { AggregateLevel } from "../main";
type IAggregateByDateFunctions = {
  [key in AggregateLevel]: AggregateFunction;
};

const aggregateFunctions = ["interval", "start", "end", "sub"] as const;

type AggregateFunctions = typeof aggregateFunctions[number];

type AggregateFunction = {
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
  },
  week: {
    interval: dateFns.eachWeekOfInterval,
    start: dateFns.startOfWeek,
    end: dateFns.endOfWeek,
    sub: dateFns.subWeeks,
  },
  month: {
    interval: dateFns.eachMonthOfInterval,
    start: dateFns.startOfMonth,
    end: dateFns.endOfMonth,
    sub: dateFns.subMonths,
  },
  year: {
    interval: dateFns.eachYearOfInterval,
    start: dateFns.startOfYear,
    end: dateFns.endOfYear,
    sub: dateFns.subYears,
  },
  hour: {
    interval: dateFns.eachHourOfInterval,
    start: dateFns.startOfHour,
    end: dateFns.endOfHour,
    sub: dateFns.subHours,
  },
  minute: {
    interval: dateFns.eachMinuteOfInterval,
    start: dateFns.startOfMinute,
    end: dateFns.endOfMinute,
    sub: dateFns.subMinutes,
  },
};
