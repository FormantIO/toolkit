import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";

ChartJS.register(...registerables);

interface IBubbleChartProps {
  labels?: string[];
  data: BubbleCoordinate[];
  height?: number | string;
  width?: number | string;
  id: string; // unique string
}

type BubbleCoordinate = {
  x: number | string;
  y: number | string;
  r: number;
};

export const BubbleChart: React.FC<IBubbleChartProps> = ({
  labels,
  data,
  height,
  width,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"bubble">>({
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

    setChartData(chartData as any);
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
    scales: {
      x: {
        grid: {
          color: "black",
          tickColor: "transparent",
        },
        ticks: {
          step: 1,
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
          color: "#bac4e2",
          font: {
            size: 9,
            family: "Atkinson Hyperlegible",
            weight: "400",
          },
        },
      },
    },
  };

  return (
    <div style={{ height: height, width: width }} className={styles.chart}>
      <Chart options={options} ref={chartRef} type="bubble" data={chartData} />
    </div>
  );
};
