import { AnalyticsAggregateType } from "./AnalyticsAggregateType";
import { AnalyticsChartType } from "./AnalyticsChartType";

export interface IAnalyticsModuleConfiguration {
  chartType?: AnalyticsChartType;
  aggregateType?: AnalyticsAggregateType;
}
