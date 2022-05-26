import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { Coordinate } from "../LineChart/LineChart";

ChartJS.register(...registerables);

interface IScatterChartProps {
  data: Coordinate[];
  height?: number | string;
  width?: number | string;
  id: string; // unique string
}

export const ScatterChart: React.FC<IScatterChartProps> = ({
  data,
  height,
  width,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"scatter">>({
    datasets: [],
  });
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) {
      return;
    }

    const chartData = {
      datasets: [
        {
          data,
          backgroundColor: ["rgba(255, 99, 132, 0.2)"],
          borderColor: ["rgba(255, 99, 132, 1)"],

          maintainAspectRatio: false,
        },
      ],
    };

    setChartData(chartData);
  }, [data]);

  const options = {
    responsive: true,
    scales: {
      x: {
        grid: {
          color: "black",
          tickColor: "transparent",
        },
        ticks: {
          stepSize: 10,
          color: "#bac4e2",
          font: {
            size: 9,
            family: "Atkinson Hyperlegible",
            weight: "400",
          },
        },
      },
      y: {
        grid: {
          color: "black",
          tickColor: "transparent",
        },
        ticks: {
          stepSize: 10,
          color: "#bac4e2",
          font: {
            size: 9,
            family: "Atkinson Hyperlegible",
            weight: "400",
          },
        },
      },
    },
    plugins: {
      legend: {},
      title: {
        display: false,
      },
    },
  };

  return (
    <div style={{ height: height, width: width }} className={styles.chart}>
      <Chart options={options} ref={chartRef} type="scatter" data={chartData} />
    </div>
  );
};
