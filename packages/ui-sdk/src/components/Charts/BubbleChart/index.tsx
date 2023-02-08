import React, { useRef, useState, useEffect } from "react";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";
import styled from "@emotion/styled";
import { ICustomTooltipParameters } from "../types";
import { Tooltip } from "../Tooltip";
ChartJS.register(...registerables);

interface IBubbleChartProps {
  data: BubbleCoordinate[];
  labels?: string[];
  height?: number | string;
  width?: number | string;
  customTooltip?: ICustomTooltipParameters;
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
  customTooltip,
}) => {
  const [_containerId] = useState(crypto.randomUUID());
  const [_labelId] = useState(crypto.randomUUID());
  const [_valueId] = useState(crypto.randomUUID());
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"bubble">>({
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
            const x = data.raw.x;
            const y = data.raw.y;
            const r = data.raw.r;
            tooltipLabel.innerHTML = label;
            tooltipValue.innerHTML = `: { x:${x} y:${y} r: ${r} }`;
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
          tooltipEl.style.padding =
            tooltipModel.padding + "px " + tooltipModel.padding + "px";
          tooltipEl!.style.pointerEvents = "none";
        },
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
    <Container style={{ height: height, width: width }}>
      {!customTooltip && (
        <Tooltip
          containerId={_containerId}
          labelId={_labelId}
          valueId={_valueId}
        />
      )}
      <Chart options={options} ref={chartRef} type="bubble" data={chartData} />
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
