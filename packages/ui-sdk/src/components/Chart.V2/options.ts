const baseOptions = {
  responsive: true,
  plugins: {
    title: {
      display: false,
    },
    legend: {
      display: false,
    },
    tooltip: {
      enabled: false,
      // external: (context: any) => {
      //   let containerId = _containerId;
      //   let labelId = _labelId;
      //   let valueId = _valueId;

      //   if (customTooltip) {
      //     containerId = customTooltip.toolTipContainerId;
      //     labelId = customTooltip.toolTipContainerId;
      //     valueId = customTooltip.toolTipYContainerId;
      //   }

      //   const tooltipEl = document.getElementById(containerId)!;
      //   tooltipEl.style.display = "flex";

      //   // Hide if no tooltip
      //   const tooltipModel = context.tooltip;
      //   if (tooltipModel.opacity === 0) {
      //     tooltipEl.style.opacity = "0";
      //     return;
      //   }

      //   // Set caret Position
      //   tooltipEl.classList.remove("above", "below", "no-transform");
      //   if (tooltipModel.yAlign) {
      //     tooltipEl.classList.add(tooltipModel.yAlign);
      //   } else {
      //     tooltipEl.classList.add("no-transform");
      //   }

      //   const tooltipLabel = document.getElementById(labelId)!;
      //   const tooltipValue = document.getElementById(valueId)!;

      //   if (tooltipModel.body) {
      //     const data = tooltipModel.dataPoints[0];
      //     const label = data.label;
      //     const value = data.parsed;
      //     tooltipLabel.innerHTML = `${label} : ${value}${tooltipUnits ?? ""}`;
      //     // tooltipValue.innerHTML = `: ${value}`;
      //   }

      //   const position = context.chart.canvas.getBoundingClientRect();

      //   // Display, position, and set styles for font
      //   tooltipEl.style.opacity = "1";
      //   tooltipEl.style.position = "absolute";
      //   tooltipEl.style.left =
      //     position.left + window.pageXOffset + tooltipModel.caretX + "px";
      //   tooltipEl.style.top =
      //     position.top + window.pageYOffset + tooltipModel.caretY + "px";
      //   tooltipEl.style.font = "Atkinson Hyperlegible";
      //   tooltipEl.style.fontSize = !!tooltipFontSize
      //     ? `${tooltipFontSize}px`
      //     : "16px";
      //   tooltipEl.style.padding =
      //     tooltipModel.padding + "px " + tooltipModel.padding + "px";
      //   tooltipEl!.style.pointerEvents = "none";
      // },
    },
  },
};
export const options = (size: string) => {
  return {
    line: null,
    bar: null,
    bubble: null,
    doughnut: {
      cutout: parseInt(size as string) / 2.5,
      ...baseOptions,
    },
    radar: null,
    scatter: null,
  };
};
