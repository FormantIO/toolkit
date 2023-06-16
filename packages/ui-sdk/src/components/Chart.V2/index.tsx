import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart as ReactChart } from "react-chartjs-2";
import React, { useRef, useState, useEffect, FC } from "react";
import { ChartProps } from "./types";
import { colors } from "./colors";

import { options } from "./options";
ChartJS.register(...registerables);

const charTypes = [
  "line",
  "bar",
  "bubble",
  "doughnut",
  "radar",
  "scatter",
] as const;

type ChartType = (typeof charTypes)[number];

interface IChart {
  type: ChartType;
  labels: string[];
}

export const Chart: FC<ChartProps> = (props: ChartProps) => {
  const { labels, data, type } = props;
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<ChartType>>({
    datasets: [],
  });

  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) {
      return;
    }

    // const tooltipEl = document.getElementById(_containerId);
    // tooltipEl!.style.display = "none";

    const chartData = {
      labels,
      datasets: [
        {
          data,
          backgroundColor: colors,
          borderColor: "#2d3855",
          borderWidth: 3,
          fill: true,
          maintainAspectRatio: false,
        },
      ],
    };

    setChartData(chartData);
  }, [data]);
  return (
    <ReactChart options={{}} ref={chartRef} type={type} data={chartData} />
  );
};
