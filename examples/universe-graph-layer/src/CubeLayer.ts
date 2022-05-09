import { UniverseLayer, Label, Hand, Controller } from "@formant/universe";
import {
  BoxGeometry,
  MeshBasicMaterial,
  Mesh,
  Raycaster,
  WebXRManager,
  CanvasTexture,
  MeshStandardMaterial,
  Color,
  Texture,
} from "three";
import { Chart, registerables } from "chart.js";
Chart.register(...registerables);

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "cube";
  static commonName = "Cube";
  static description = "This is just a simple cube.";

  geo = new BoxGeometry(1, 1, 1);
  mat = new MeshStandardMaterial();
  cube = new Mesh(this.geo, this.mat);

  label = new Label("controller");

  init() {
    const canvas = document.createElement("canvas");
    canvas.style.display = "none";
    canvas.width = canvas.height = 1000;
    document.body.appendChild(canvas);
    const ctx = canvas.getContext("2d");
    if (ctx) {
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
      myChart.update();
      window.setTimeout(() => {
        ctx.fillStyle = "rgb(255,0,0)";
        ctx.fillRect(0, 0, 40, 40);
        const canvasTex = new Texture(myChart.canvas);
        canvasTex.needsUpdate = true;

        this.mat.map = canvasTex;
        this.mat.emissive = new Color(0x344444);
        this.mat.needsUpdate = true;
      }, 1000);
    }
    this.add(this.cube);
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
