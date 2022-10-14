import { INumericAggregate } from "./INumericAggregate";

export type INumericSetAggregateMap = Map<
  string,
  {
    value: INumericAggregate;
    unit?: string;
  }
>;
