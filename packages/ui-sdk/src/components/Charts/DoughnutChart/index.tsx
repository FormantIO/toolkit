import React, { useRef, useState, useEffect, FC } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";

ChartJS.register(...registerables);

interface IChartProps {
  size: string | number;
  labels: string[];
  data: number[];
}

export const DoughnutChart: FC<IChartProps> = ({ size, labels, data }) => {
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

  const options = {
    responsive: true,
    cutout: parseInt(size as string) / 2.5,
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
    <div
      style={{ height: size, width: size }}
      className={styles["formant-doughnut-chart"]}
    >
      <Chart
        options={options}
        ref={chartRef}
        type="doughnut"
        data={chartData}
      />
    </div>
  );
};
