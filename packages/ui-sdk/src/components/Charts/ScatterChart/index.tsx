import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { Coordinate } from "../LineChart/LineChart";
import { colors } from "../colors";
ChartJS.register(...registerables);

interface IScatterChartProps {
  data: Coordinate[];
  height?: number | string;
  width?: number | string;
  id: string; // unique string
  xMax?: number;
  xMin?: number;
  yMax?: number;
  yMin?: number;
}

export const ScatterChart: React.FC<IScatterChartProps> = ({
  data,
  height,
  width,
  xMax,
  xMin,
  yMax,
  yMin,
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
          backgroundColor: colors.map((_) => _ + "33") as string[],
          borderColor: colors[3],

          maintainAspectRatio: false,
        },
      ],
    };

    setChartData(chartData as any);
  }, [data]);

  const options = {
    responsive: true,
    scales: {
      x: {
        min: xMin ?? 0,
        max: xMax ?? 100,
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
        min: yMin ?? 0,
        max: yMax ?? 100,
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
      legend: { display: false },
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
