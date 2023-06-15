import { ICustomTooltipParameters } from "../types";

export const getOptions = (
  data: { x: number; y: number }[],
  color: string,
  _containerId: string,
  _labelId: string,
  _valueId: string,
  Ymin?: number,
  Ymax?: number,
  customTooltip?: ICustomTooltipParameters
) => {
  const getXaxisandYaxisMinAndMax = () => {
    let Xmax: number = data[0].x;
    let Xmin: number = data[0].x;

    data.map((_, idx) => {
      if (idx === data.length) return;
      if (data[idx].x > Xmax) Xmax = data[idx].x;
      if (data[idx].x < Xmin) Xmin = data[idx].x;
    });

    return {
      Xmax,
      Xmin,
      Ymax: Ymax ?? 100,
      Ymin: Ymin ?? 0,
    };
  };

  const scaleLimits = getXaxisandYaxisMinAndMax();

  return {
    maintainAspectRatio: false,
    elements: {
      animation: false,
      point: {
        pointBackgroundColor: "transparent",
        pointBorderColor: "transparent",
        radius: 5, // Erase dots from sccater plot
        hoverRadius: 5,
        pointHoverBorderColor: color,
      },
    },
    responsive: true,
    plugins: {
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
            const data = tooltipModel.dataPoints[0].raw;
            const label = data.label;
            const x = data.x;
            const y = data.y;
            tooltipLabel.innerHTML = label ?? "";
            tooltipValue.innerHTML = `x: ${x}  y: ${y}`;
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
      legend: {
        display: false, //Name and color of dataset
      },
    },
    scales: {
      x: {
        display: false, //Grid Lines
        min: scaleLimits.Xmin,
        max: scaleLimits.Xmax,
      },
      y: {
        min: -10,
        max: Math.ceil(scaleLimits.Ymax / 10) * 10,
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
    labels: {
      fontSize: 9,
    },
  };
};
