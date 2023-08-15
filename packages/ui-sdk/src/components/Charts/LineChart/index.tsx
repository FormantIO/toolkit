import React, { useRef, useState, useEffect } from "react";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { getOptions } from "./getOptions";
import { getGradient } from "./getGradient";
import styled from "@emotion/styled";
import { Tooltip } from "../Tooltip";
import { ICustomTooltipParameters } from "../types";

ChartJS.register(...registerables);

const borderLineWidth = 1;

export type Coordinate = { x: number; y: number };

interface ILineChartProps {
  data: Coordinate[];
  color: string;
  height?: number;
  width?: number;
  Ymin?: number;
  Ymax?: number;
  customTooltip?: ICustomTooltipParameters;
}

export const LineChart: React.FC<ILineChartProps> = ({
  data,
  color,
  customTooltip,
  height,
  width,
  Ymax,
  Ymin,
}) => {
  const [_containerId] = useState(Math.random().toString(36));
  const [_labelId] = useState(Math.random().toString(36));
  const [_valueId] = useState(Math.random().toString(36));
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"line">>({
    datasets: [],
  });
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) {
      return;
    }
    const tooltipEl = document.getElementById(_containerId);
    tooltipEl!.style.display = "none";
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
    <Container style={{ height: height, width: width }}>
      {!customTooltip && (
        <Tooltip
          containerId={_containerId}
          labelId={_labelId}
          valueId={_valueId}
        />
      )}
      <Chart
        id="Line"
        options={getOptions(
          data,
          color,
          _containerId,
          _labelId,
          _valueId,
          Ymin,
          Ymax,
          customTooltip
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
    </Container>
  );
};

const Container = styled.div`
  width: 70vw;
  height: 90vh;
  canvas {
    height: 100%;
    width: 100%;
    &:hover {
      cursor: pointer;
    }
  }
`;
