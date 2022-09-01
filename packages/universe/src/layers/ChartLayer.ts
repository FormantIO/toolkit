import { Chart, registerables } from "chart.js";
import { Mesh, CanvasTexture, PlaneGeometry, MeshBasicMaterial } from "three";
import { defined, INumericSetEntry, UniverseLayer } from "../main";

export const colors = [
  "#2EC495",
  "#20A0FF",
  "#A961E4",
  "#EA719D",
  "#F89973",
  "#F9C36E",
];

Chart.register(...registerables);

export class ChartLayer extends UniverseLayer {
  static layerTypeId = "chart";

  static commonName = "Graph";

  static description = "This is just a simple graph.";

  geo!: PlaneGeometry;

  mat = new MeshBasicMaterial();

  cube!: Mesh;

  canvasTex!: CanvasTexture;

  didChange: boolean = false;

  chart!: Chart;

  static fields = {
    width: {
      name: "Width",
      description: "The width of the chart",
      placeholder: 1,
      value: 1,
      type: "number" as const,
      location: ["create" as const],
    },
    height: {
      name: "Height",
      description: "The height of the chart",
      placeholder: 1,
      value: 1,
      type: "number" as const,
      location: ["create" as const],
    },
  };

  init() {
    const width = this.getField(ChartLayer.fields.width) || 1;
    const height = this.getField(ChartLayer.fields.height) || 1;
    this.geo = new PlaneGeometry(width, height);
    this.cube = new Mesh(this.geo, this.mat)
      .rotateY(Math.PI / 2)
      .rotateZ(Math.PI / 2);
    const canvas = document.createElement("canvas");
    canvas.style.display = "none";
    canvas.width = 1000;
    canvas.height = (canvas.width * height) / width;
    document.body.appendChild(canvas);
    const ctx = canvas.getContext("2d");
    if (ctx) {
      Chart.defaults.font.size = 30;
      const plugin = {
        id: "custom_canvas_background_color",
        beforeDraw: (chart: any) => {
          const ctx2 = chart.canvas.getContext("2d");
          ctx2.save();
          ctx2.globalCompositeOperation = "destination-over";
          ctx2.fillStyle = "#2d3855";
          ctx2.fillRect(0, 0, chart.width, chart.height);
          ctx2.restore();
        },
      };
      Chart.defaults.color = "#bac4e2";
      this.chart = new Chart(ctx, {
        type: "line",
        data: {
          datasets: [],
        },
        options: {
          scales: {
            y: {
              type: "linear",
              beginAtZero: false,
            },
            x: {
              type: "linear",
              beginAtZero: false,
              ticks: {
                callback: (value) => {
                  const date = new Date((value as number) * 1000);
                  const hours = date.getHours();
                  const minutes = `0${date.getMinutes()}`;
                  const seconds = `0${date.getSeconds()}`;
                  const formattedTime = `${hours}:${minutes.substr(
                    -2
                  )}:${seconds.substr(-2)}`;
                  return formattedTime;
                },
              },
            },
          },
          animation: {
            duration: 0,
          },
        },
        plugins: [plugin],
      });
      this.canvasTex = new CanvasTexture(this.chart.canvas);
      this.mat.map = this.canvasTex;
      this.didChange = true;
    }
    this.add(this.cube);
    this.universeData.subscribeToNumericSet(
      defined(this.getLayerContext()).deviceId,
      defined(this.layerDataSources)[0],
      (d) => {
        if (typeof d === "symbol") {
          throw new Error("unhandled data status");
        }
        this.onData(d as [number, INumericSetEntry[]][]);
      }
    );
  }

  onData(newData: [number, INumericSetEntry[]][]): void {
    const labeledData: { [key in string]: { x: number; y: number }[] } = {};
    newData.forEach(([time, data]) => {
      data.forEach((_) => {
        if (labeledData[_.label]) {
          labeledData[_.label].push({ x: time, y: _.value });
        } else {
          labeledData[_.label] = [{ x: time, y: _.value }];
        }
      });
    });
    const { data } = this.chart;

    if (data) {
      const dataSets = Object.keys(labeledData).map((_, i) => ({
        label: _,
        data: labeledData[_].map((d) => ({ x: d.x, y: d.y })),
        borderColor: colors[i],
        backgroundColor: "transparent",
        borderWidth: 3,
        tension: 0.5,
        pointRadius: 0,
        pointHoverRadius: 0,
        fill: "+1",
      }));

      data.datasets = dataSets;
    }
    this.chart.update();
    this.didChange = true;
  }

  onUpdate(_delta: number): void {
    if (this.canvasTex && this.didChange) {
      this.canvasTex.needsUpdate = true;
      this.didChange = false;
    }
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
