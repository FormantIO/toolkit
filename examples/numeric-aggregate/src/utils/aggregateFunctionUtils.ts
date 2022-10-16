import { INumericAggregate } from "@formant/data-sdk";

function getVariance(a: INumericAggregate) {
  if (a.count < 2) {
    return 0;
  }
  return a.sumOfSquares / (a.count - 1);
}

function getStandardDeviation(a: INumericAggregate) {
  return Math.sqrt(getVariance(a));
}

function getMax(a: INumericAggregate) {
  return a.max;
}

function getMin(a: INumericAggregate) {
  return a.min;
}

function getAverage(a: INumericAggregate) {
  return a.count === 0 ? -1 : a.sum / a.count;
}

function getSum(a: INumericAggregate) {
  return a.sum;
}
function getCount(a: INumericAggregate) {
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
