import React, { useRef, useState, useEffect } from "react";
import { Chart as ChartJS, ChartData, registerables } from "chart.js";
import { Chart } from "react-chartjs-2";
import { colors } from "../colors";
import styled from "@emotion/styled";
import { Tooltip } from "../Tooltip";
import { ICustomTooltipParameters } from "../types";

ChartJS.register(...registerables);

interface IBarChartProps {
  labels: string[] | string[][];
  data: number[];
  height?: number;
  width?: number;
  xMax?: number;
  xMin?: number;
  showYGrid?: boolean;
  showXGrid?: boolean;
  customTooltip?: ICustomTooltipParameters;
  xTicksFontSize?: number;
  tooltipFontSize?: number;
  XAxisMaxLengthLabel?: number;
  tooltipUnits?: string;
}

export const BarChart: React.FC<IBarChartProps> = ({
  labels,
  data,
  height,
  width,
  xMax,
  xMin,
  showXGrid,
  showYGrid,
  customTooltip,
  xTicksFontSize,
  tooltipFontSize,
  XAxisMaxLengthLabel,
  tooltipUnits,
}) => {
  const [_containerId] = useState(Math.random().toString(36));
  const [_labelId] = useState(Math.random().toString(36));
  const [_valueId] = useState(Math.random().toString(36));
  const chartRef = useRef<ChartJS>(null);
  const [chartData, setChartData] = useState<ChartData<"bar">>({
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
            let label: string = data.label;
            const value = data.raw;

            tooltipLabel.innerHTML = `${label}: ${value}${tooltipUnits ?? ""}`;
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
      title: {
        display: false,
      },
    },
    barPercentage: 0.12,
    scales: {
      x: {
        grid: {
          color: showXGrid == undefined || showXGrid ? "black" : "transparent",
          tickColor: "transparent",
        },
        ticks: {
          color: "#bac4e2",
          font: {
            size: xTicksFontSize ?? 14,
            family: "inter",
            weight: "400",
          },
          callback: (t: number) => {
            let l = 10;
            if (!!XAxisMaxLengthLabel) {
              l = XAxisMaxLengthLabel;
            }

            return labels[t].length > l
              ? `${labels[t].slice(0, l - 4)}...`
              : labels[t];
          },
        },
      },
      y: {
        min: xMin ?? 0,
        max: xMax ?? 100,
        grid: {
          color:
            showYGrid == undefined || showYGrid ? "#1C1E2D" : "transparent",
          tickColor: "transparent",
        },
        ticks: {
          color: "#bac4e2",
          font: {
            size: 9,
            family: "inter",
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
