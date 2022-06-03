import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";

ChartJS.register(...registerables);

interface IRadarChartProps {
  labels: string[];
  data: number[];
  height?: number | string;
  width?: number | string;
  id: string; // unique string
}

export const RadarChart: React.FC<IRadarChartProps> = ({
  labels,
  data,
  height,
  width,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"radar">>({
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

    scales: {
      r: {
        angleLines: {
          color: "black",
        },
        grid: {
          color: "black",
        },
        pointLabels: {
          color: "#bac4e2",
        },
        ticks: {
          color: "#bac4e2",
          backdropColor: "#2d3855",
          font: {
            family: "Atkinson Hyperlegible",

            weight: "400",
          },
        },
      },
    },
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
      <Chart options={options} ref={chartRef} type="radar" data={chartData} />
    </div>
  );
};
