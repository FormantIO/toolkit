import { UniverseLayer, Label } from "@formant/universe";
import {
  BoxGeometry,
  Mesh,
  CanvasTexture,
  MeshStandardMaterial,
  Color,
} from "three";
import { Chart, registerables } from "chart.js";
Chart.register(...registerables);

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "graph";
  static commonName = "Graph";
  static description = "This is just a simple graph.";

  geo = new BoxGeometry(1, 0, 1);
  mat = new MeshStandardMaterial();
  cube = new Mesh(this.geo, this.mat).rotateY(Math.PI);

  label = new Label("controller");
  canvasTex!: CanvasTexture;

  init() {
    const canvas = document.createElement("canvas");
    canvas.style.display = "none";
    canvas.width = canvas.height = 1000;
    document.body.appendChild(canvas);
    const ctx = canvas.getContext("2d");
    if (ctx) {
      Chart.defaults.font.size = 30;
      const myChart = new Chart(ctx, {
        type: "bar",
        data: {
          labels: ["Red", "Blue", "Yellow", "Green", "Purple", "Orange"],
          datasets: [
            {
              label: "# of Votes",
              data: [12, 19, 3, 5, 2, 3],
              backgroundColor: [
                "rgba(255, 99, 132, 0.2)",
                "rgba(54, 162, 235, 0.2)",
                "rgba(255, 206, 86, 0.2)",
                "rgba(75, 192, 192, 0.2)",
                "rgba(153, 102, 255, 0.2)",
                "rgba(255, 159, 64, 0.2)",
              ],
              borderColor: [
                "rgba(255, 99, 132, 1)",
                "rgba(54, 162, 235, 1)",
                "rgba(255, 206, 86, 1)",
                "rgba(75, 192, 192, 1)",
                "rgba(153, 102, 255, 1)",
                "rgba(255, 159, 64, 1)",
              ],
              borderWidth: 1,
            },
          ],
        },
        options: {
          scales: {
            y: {
              beginAtZero: true,
            },
          },
        },
      });
      this.canvasTex = new CanvasTexture(myChart.canvas);
      this.mat.map = this.canvasTex;
      this.mat.emissive = new Color(0x444444);
    }
    this.add(this.cube);
  }

  onUpdate(_delta: number): void {
    this.canvasTex.needsUpdate = true;
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
