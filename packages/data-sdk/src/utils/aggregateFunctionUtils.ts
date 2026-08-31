import { INumericAggregate } from "../model/INumericAggregate";
import * as dateFns from "date-fns";
import { AggregateLevel } from "../model/AggregateLevel";

export const vailableAggregationIntervals = [
  "day",
  "week",
  "month",
  "year",
  "hour",
  "minute",
  "quarter",
] as const;

export type ValidAggregationInterval =
  (typeof vailableAggregationIntervals)[number];

export type IAggregateByDateFunctions = {
  [key in ValidAggregationInterval]: AggregateFunction;
};

export const aggregateFunctions = [
  "interval",
  "start",
  "end",
  "sub",
  "get",
] as const;

export type AggregateFunctions = (typeof aggregateFunctions)[number];

export type AggregateFunction = IAggregateFunctions;

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

interface IAggregateFunctions {
  interval: (interval: dateFns.Interval) => Date[];
  start: (date: Date | number) => Date;
  end: (date: Date | number) => Date;
  sub: (date: Date | number, amount: number) => Date;
  get: (date: Date | number) => number;
}

export const aggregateByDateFunctions: Record<
  AggregateLevel,
  IAggregateFunctions
> = {
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

export type TimeFrameBoundary = Date | number | string;

type LocalizedDateField = "year" | "month" | "day";

const dayAndMonthOptions: Intl.DateTimeFormatOptions = {
  month: "numeric",
  day: "numeric",
};

// Day, month and year are all distinct here so that the locale's field order
// can be read back off the formatted parts without any of them being ambiguous.
const fieldOrderProbe = new Date(Date.UTC(2020, 10, 25, 12));

const getDateFieldOrder = (locales?: string | string[]) =>
  new Intl.DateTimeFormat(locales)
    .formatToParts(fieldOrderProbe)
    .reduce<LocalizedDateField[]>((fields, part) => {
      if (
        part.type === "year" ||
        part.type === "month" ||
        part.type === "day"
      ) {
        fields.push(part.type);
      }
      return fields;
    }, []);

const toAsciiDigits = (value: string, locales?: string | string[]) => {
  const numberFormat = new Intl.NumberFormat(locales, { useGrouping: false });
  const asciiByLocalizedDigit = new Map<string, string>();
  for (let digit = 0; digit <= 9; digit += 1) {
    asciiByLocalizedDigit.set(numberFormat.format(digit), String(digit));
  }
  return Array.from(value)
    .map((character) => asciiByLocalizedDigit.get(character) ?? character)
    .join("");
};

const parseLocalizedDate = (
  value: string,
  locales?: string | string[]
): Date | undefined => {
  const fieldOrder = getDateFieldOrder(locales);
  const numbers = toAsciiDigits(value, locales).match(/\d+/g);
  if (!numbers || numbers.length < fieldOrder.length) {
    return undefined;
  }

  const fields: Partial<Record<LocalizedDateField, number>> = {};
  fieldOrder.forEach((field, index) => {
    fields[field] = Number(numbers[index]);
  });

  const { year, month, day } = fields;
  if (year === undefined || month === undefined || day === undefined) {
    return undefined;
  }

  // A date that rolled over (13th month, 30th of February) means the string
  // never matched the locale, so it is rejected instead of silently shifted.
  const parsed = new Date(year, month - 1, day);
  return parsed.getMonth() === month - 1 && parsed.getDate() === day
    ? parsed
    : undefined;
};

const toDate = (
  boundary: TimeFrameBoundary,
  locales?: string | string[]
): Date | undefined => {
  if (typeof boundary === "string") {
    return parseLocalizedDate(boundary, locales);
  }
  const date = boundary instanceof Date ? boundary : new Date(boundary);
  return Number.isNaN(date.getTime()) ? undefined : date;
};

const formatDayAndMonth = (
  boundary: TimeFrameBoundary,
  locales?: string | string[]
) => {
  const date = toDate(boundary, locales);
  if (date === undefined) {
    return typeof boundary === "string" ? boundary : "";
  }
  return new Intl.DateTimeFormat(locales, dayAndMonthOptions).format(date);
};

export const formatTimeFrameText = (
  start: TimeFrameBoundary,
  end: TimeFrameBoundary,
  locales?: string | string[]
) => `${formatDayAndMonth(start, locales)}–${formatDayAndMonth(end, locales)}`;
