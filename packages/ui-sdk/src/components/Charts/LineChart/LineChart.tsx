import React, { useRef, useState, useEffect } from "react";
import styles from "./LineChart.module.scss";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { getOptions } from "./getOptions";
import { getGradient } from "./getGradient";

ChartJS.register(...registerables);

const borderLineWidth = 1;

export type Coordinate = { x: number; y: number };

interface ILineChartProps {
  data: Coordinate[];
  color: string;
  CustomTooltip?: React.FC;
  toolTipContainerId?: string;
  toolTipXContainerId?: string;
  toolTipYContainerId?: string;
  height?: number;
  width?: number;
}

export const LineChart: React.FC<ILineChartProps> = ({
  data,
  color,
  CustomTooltip,
  toolTipContainerId,
  toolTipXContainerId,
  toolTipYContainerId,
  height,
  width,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"line">>({
    datasets: [],
  });
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) {
      return;
    }
    if (!!toolTipContainerId) {
      let tooltipEl = document.getElementById(toolTipContainerId);
      tooltipEl!.style.display = "none";
    }
    const chartData = {
      datasets: [
        {
          data,
          backgroundColor: getGradient(color, chart.ctx, height),
          borderColor: color,
          fill: true,
          showLine: true,
          borderWidth: 2,
          maintainAspectRatio: false,
          tension: 0.5,
        },
      ],
    };

    setChartData(chartData);
  }, [data]);

  return (
    <div style={{ height: height, width: width }} className={styles.chart}>
      {!!CustomTooltip && <CustomTooltip />}
      <Chart
        id="Line"
        options={getOptions(
          data,
          color,
          toolTipContainerId,
          toolTipXContainerId,
          toolTipYContainerId
        )}
        ref={chartRef}
        type="scatter"
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
              ctx.lineWidth = borderLineWidth;
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
