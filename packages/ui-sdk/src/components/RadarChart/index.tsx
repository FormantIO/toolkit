import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";

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
          backgroundColor: [
            "rgba(255, 99, 132, 0.2)",
            "rgba(54, 162, 235, 0.2)",
            "rgba(255, 206, 86, 0.2)",
            "rgba(75, 192, 192, 0.2)",
            "rgba(153, 102, 255, 0.2)",
            "rgba(255, 159, 64, 0.2)",
          ],
          borderColor: [
            "rgba(255, 99, 132, 1)",
            "rgba(54, 162, 235, 1)",
            "rgba(255, 206, 86, 1)",
            "rgba(75, 192, 192, 1)",
            "rgba(153, 102, 255, 1)",
            "rgba(255, 159, 64, 1)",
          ],
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
      legend: {},
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
