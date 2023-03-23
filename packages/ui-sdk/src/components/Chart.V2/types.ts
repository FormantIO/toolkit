const charTypes = [
  "line",
  "bar",
  "bubble",
  "doughnut",
  "radar",
  "scatter",
] as const;

type ChartType = typeof charTypes[number];

export interface IChart<T extends ChartType> {
  type: T;
  labels: string[];
}

export interface IDoughnutProps extends IChart<"doughnut"> {
  data: number[];
}

export type ChartProps = IDoughnutProps;
