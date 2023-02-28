import { INumericSetConfiguration } from "../types";

export const dummyData: INumericSetConfiguration = {
  fullScreenMode: true,
  streamType: "numeric set",
  aggregateType: "average",
  numericSetKey: "utilization",
  aggregateBy: "day",
  numAggregates: 5,
  numericSetStream: "$.host.cpu",
};
