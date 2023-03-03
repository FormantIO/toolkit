import React, { useRef, useState, useEffect, FC } from "react";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";
import styled from "@emotion/styled";
import { Card } from "./Card";
import { ICustomTooltipParameters } from "../types";

ChartJS.register(...registerables);

interface IChartProps {
  size: string | number;
  labels: string[];
  data: number[];
  customTooltip?: ICustomTooltipParameters;
  tooltipFontSize?: number;
}

export const DoughnutChart: FC<IChartProps> = ({
  size,
  labels,
  data,
  customTooltip,
  tooltipFontSize,
}) => {
  const chartRef = useRef<ChartJS>(null);
  const [_containerId] = useState(crypto.randomUUID());
  const [_labelId] = useState(crypto.randomUUID());
  const [_valueId] = useState(crypto.randomUUID());
  const [chartData, setChartData] = useState<ChartData<"doughnut">>({
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
      tooltip: {
        enabled: false,
        external: (context: any) => {
          let containerId = _containerId;
          let labelId = _labelId;
          let valueId = _valueId;

          if (customTooltip) {
            containerId = customTooltip.toolTipContainerId;
            labelId = customTooltip.toolTipContainerId;
            valueId = customTooltip.toolTipYContainerId;
          }

          const tooltipEl = document.getElementById(containerId)!;
          tooltipEl.style.display = "flex";

          // Hide if no tooltip
          const tooltipModel = context.tooltip;
          if (tooltipModel.opacity === 0) {
            tooltipEl.style.opacity = "0";
            return;
          }

          // Set caret Position
          tooltipEl.classList.remove("above", "below", "no-transform");
          if (tooltipModel.yAlign) {
            tooltipEl.classList.add(tooltipModel.yAlign);
          } else {
            tooltipEl.classList.add("no-transform");
          }

          const tooltipLabel = document.getElementById(labelId)!;
          const tooltipValue = document.getElementById(valueId)!;

          if (tooltipModel.body) {
            const data = tooltipModel.dataPoints[0];
            const label = data.label;
            const value = data.parsed;
            tooltipLabel.innerHTML = `${label} : ${value}`;
            // tooltipValue.innerHTML = `: ${value}`;
          }

          const position = context.chart.canvas.getBoundingClientRect();

          // Display, position, and set styles for font
          tooltipEl.style.opacity = "1";
          tooltipEl.style.position = "absolute";
          tooltipEl.style.left =
            position.left + window.pageXOffset + tooltipModel.caretX + "px";
          tooltipEl.style.top =
            position.top + window.pageYOffset + tooltipModel.caretY + "px";
          tooltipEl.style.font = "Atkinson Hyperlegible";
          tooltipEl.style.fontSize = !!tooltipFontSize
            ? `${tooltipFontSize}px`
            : "16px";
          tooltipEl.style.padding =
            tooltipModel.padding + "px " + tooltipModel.padding + "px";
          tooltipEl!.style.pointerEvents = "none";
        },
      },
    },
  };
  return (
    <Container style={{ height: size, width: size }}>
      {!customTooltip && (
        <Card
          containerId={_containerId}
          labelId={_labelId}
          valueId={_valueId}
        />
      )}
      <Chart
        options={options}
        ref={chartRef}
        type="doughnut"
        data={chartData}
      />
    </Container>
  );
};

const Container = styled.div`
  canvas {
    height: 100%;
    width: 100%;
    &:hover {
      cursor: pointer;
    }
  }
`;
