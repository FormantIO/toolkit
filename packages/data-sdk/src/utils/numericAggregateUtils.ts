import {
  INumericAggregate,
  INumericSetAggregateMap,
  IStreamAggregateData,
  IStreamTypeMap,
} from "../main";

export type WireINumericSetAggregateMap = {
  [id: string]: {
    value: INumericAggregate;
    unit?: string;
  };
};

export function getZeroINumericSet() {
  return {
    min: Number.MAX_SAFE_INTEGER,
    max: 0,
    sum: 0,
    count: 0,
    sumOfSquares: 0,
  };
}

export function reduceNumericStreamAggregates(
  stream: IStreamAggregateData<keyof IStreamTypeMap>
) {
  return stream.aggregates.reduce((acc, entry) => {
    const aggregate = entry[1] as INumericAggregate;
    return combineNumericAggregates(aggregate, acc);
  }, getZeroINumericSet());
}

export function reduceNumericSetStreamAggregates(
  stream: IStreamAggregateData<keyof IStreamTypeMap>,
  numericSetKey: string
) {
  return stream.aggregates.reduce((acc, entry) => {
    const aggregate = entry[1] as INumericSetAggregateMap;
    return combineNumericSetAggregates(
      aggregate as unknown as WireINumericSetAggregateMap,
      acc
    );
  }, {} as WireINumericSetAggregateMap)[numericSetKey]?.value;
}

export function combineNumericAggregates(
  e1: INumericAggregate,
  e2: INumericAggregate
) {
  return {
    min: Math.min(e1.min, e2.min),
    max: Math.max(e1.max, e2.max),
    sum: e1.sum + e2.sum,
    count: e1.count + e2.count,
    sumOfSquares: e1.sumOfSquares + e2.sumOfSquares,
  } as INumericAggregate;
}

export function combineNumericSetAggregates(
  e1: WireINumericSetAggregateMap,
  e2: WireINumericSetAggregateMap
) {
  return Object.keys(e1).reduce((acc, key) => {
    return {
      ...acc,
      [key]: {
        value: combineNumericAggregates(
          e1[key].value,
          e2[key]?.value ?? getZeroINumericSet()
        ),
        unit: e1[key].unit,
      },
    };
  }, {} as WireINumericSetAggregateMap);
}
