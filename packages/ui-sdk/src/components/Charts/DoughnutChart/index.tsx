import React, { useRef, useState, useEffect, FC } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";

ChartJS.register(...registerables);

interface IChartProps {
  height: string | number;
  width: string | number;
  labels: string[];
  data: number[];
}

export const DoughnutChart: FC<IChartProps> = ({
  height,
  width,
  labels,
  data,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"doughnut">>({
    datasets: [],
  });
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) {
      return;
    }

    const chartData = {
      labels,
      datasets: [
        {
          data,
          backgroundColor: colors.map((_) => _ + "33") as string[],
          borderColor: colors,
          fill: true,
          maintainAspectRatio: false,
        },
      ],
    };

    setChartData(chartData);
  }, [data]);

  const options = {
    responsive: true,
    plugins: {
      legend: {
        display: false,
      },
      title: {
        display: false,
      },
    },
  };
  return (
    <div style={{ height: height, width: width }} className={styles.chart}>
      <Chart
        options={options}
        ref={chartRef}
        type="doughnut"
        data={chartData}
      />
    </div>
  );
};
