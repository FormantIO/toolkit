import React, { useRef, useState, useEffect } from "react";
import styles from "./index.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";

ChartJS.register(...registerables);

interface IBarChartProps {
  labels: string[];
  data: number[];
  height?: number;
  width?: number;
  xMax?: number;
  xMin?: number;
}

export const BarChart: React.FC<IBarChartProps> = ({
  labels,
  data,
  height,
  width,
  xMax,
  xMin,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"bar">>({
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
    barPercentage: 0.12,
    scales: {
      x: {
        grid: {
          color: "black",
          tickColor: "transparent",
        },
        ticks: {
          color: "#bac4e2",
          font: {
            size: 16,
            family: "Atkinson Hyperlegible",
            weight: "400",
          },
        },
      },
      y: {
        min: xMin ?? 0,
        max: xMax ?? 100,
        grid: {
          color: "#1C1E2D",
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
      <Chart
        // id="Line"
        options={options}
        ref={chartRef}
        type="bar"
        data={chartData}
        plugins={[
          {
            id: "customBorder",
            beforeDatasetDraw(chart) {
              const {
                ctx,
                chartArea: { top, bottom, left, right },
              } = chart;
              ctx.save();
              ctx.beginPath();
              ctx.lineWidth = 1;
              ctx.moveTo(left, top);
              ctx.lineTo(right, top);
              ctx.lineTo(right, bottom);
              ctx.lineTo(left, bottom);
              ctx.closePath();
              ctx.stroke();
            },
          },
        ]}
      />
    </div>
  );
};
