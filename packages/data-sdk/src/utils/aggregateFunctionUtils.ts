import { INumericAggregate } from "../model/INumericAggregate";

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
