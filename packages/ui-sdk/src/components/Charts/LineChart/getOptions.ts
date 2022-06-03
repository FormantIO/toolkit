export const getOptions = (
  data: { x: number; y: number }[],
  color: string,
  toolTipContainerId?: string,
  toolTipXContainerId?: string,
  toolTipYContainerId?: string
) => {
  const getXaxisandYaxisMinAndMax = () => {
    let Xmax: number = data[0].x;
    let Xmin: number = data[0].x;
    let Ymax: number = data[0].y;
    let Ymin: number = data[0].y;

    data.map((_, idx) => {
      if (idx === data.length) return;
      if (data[idx].x > Xmax) Xmax = data[idx].x;
      if (data[idx].x < Xmin) Xmin = data[idx].x;
      if (data[idx].y > Ymax) Ymax = data[idx].y;
      if (data[idx].y < Ymin) Ymin = data[idx].y;
    });

    return {
      Xmax,
      Xmin,
      Ymax,
      Ymin,
    };
  };

  const scaleLimits = getXaxisandYaxisMinAndMax();

  return {
    maintainAspectRatio: false,

    elements: {
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
        enabled: !!toolTipContainerId ? false : true,
        external: (context: any) => {
          // Tooltip Element
          if (!!!toolTipContainerId) return;
          let tooltipEl = document.getElementById(toolTipContainerId);
          tooltipEl!.style.display = "flex";

          // Hide if no tooltip
          const tooltipModel = context.tooltip;
          if (tooltipModel.opacity === 0) {
            tooltipEl!.style.opacity = "0";
            return;
          }

          // Set caret Position
          tooltipEl!.classList.remove("above", "below", "no-transform");
          if (tooltipModel.yAlign) {
            tooltipEl!.classList.add(tooltipModel.yAlign);
          } else {
            tooltipEl!.classList.add("no-transform");
          }

          let xContainer = document.getElementById(toolTipXContainerId!);
          let yContainer = document.getElementById(toolTipYContainerId!);

          if (tooltipModel.body) {
            const coordinate = tooltipModel.body[0].lines[0];
            let xindex = coordinate.indexOf(",");
            let yindex = coordinate.indexOf(")");
            const x = coordinate.slice(1, xindex);
            const y = coordinate.slice(xindex + 1, yindex);
            xContainer!.innerHTML = x;
            yContainer!.innerHTML = y;
          }

          const position = context.chart.canvas.getBoundingClientRect();

          // Display, position, and set styles for font
          tooltipEl!.style.opacity = "1";
          tooltipEl!.style.position = "absolute";
          tooltipEl!.style.left =
            position.left + window.pageXOffset + tooltipModel.caretX + "px";
          tooltipEl!.style.top =
            position.top + window.pageYOffset + tooltipModel.caretY + "px";
          tooltipEl!.style.font = "Atkinson Hyperlegible";
          tooltipEl!.style.padding =
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
