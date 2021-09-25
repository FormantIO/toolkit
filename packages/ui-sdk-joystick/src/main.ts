import { defined, definedAndNotNull } from "../../common/defined";

export class Joystick extends HTMLElement {
  ctx?: CanvasRenderingContext2D;
  size = 100;
  x = 0;
  y = 0;

  constructor() {
    super();
  }
  connectedCallback() {
    this.innerHTML = `<canvas width="${this.size}" height="${this.size}"></canvas>`;
    const canvas = definedAndNotNull(this.querySelector("canvas"));
    this.ctx = definedAndNotNull(canvas.getContext("2d"));
    if (this.getAttribute("wasd") !== null) {
      document.addEventListener("keydown", (e) => {
        console.log(e);
        if (e.key === "w") {
          this.updatePosition(null, 1);
        } else if (e.key === "a") {
          this.updatePosition(-1, null);
        } else if (e.key === "s") {
          this.updatePosition(null, -1);
        } else if (e.key === "d") {
          this.updatePosition(1, null);
        }
        this.render();
      });
      document.addEventListener("keyup", (e) => {
        console.log(e);
        if (e.key === "w") {
          this.updatePosition(null, 0);
        } else if (e.key === "a") {
          this.updatePosition(0, null);
        } else if (e.key === "s") {
          this.updatePosition(null, 0);
        } else if (e.key === "d") {
          this.updatePosition(0, null);
        }
        this.render();
      });
    }
    this.render();
  }

  updatePosition(x: number | null, y: number | null) {
    if (x !== null) {
      this.x = x;
    }
    if (y !== null) {
      this.y = y;
    }
    const event = new CustomEvent("joystick", {
      detail: { x: this.x, y: this.y },
    });
    this.dispatchEvent(event);
  }

  render() {
    const ctx = defined(this.ctx);

    ctx.clearRect(0, 0, this.size, this.size);
    ctx.beginPath();
    ctx.arc(
      this.size / 2,
      this.size / 2,
      (this.size / 2) * 0.8,
      0,
      2 * Math.PI
    );
    ctx.globalAlpha = 0.5;
    ctx.fillStyle = "black";
    ctx.fill();
    ctx.globalAlpha = 1;
    var grad = ctx.createLinearGradient(50, 50, 150, 150);
    grad.addColorStop(0, "#aa9eff");
    grad.addColorStop(1, "#14dbff");
    ctx.lineWidth = 2;
    ctx.strokeStyle = grad;
    ctx.stroke();
    ctx.fillStyle = "white";
    ctx.beginPath();
    ctx.arc(
      this.size / 2 + (this.x * this.size * 0.8) / 2,
      this.size / 2 + (-1 * this.y * this.size * 0.8) / 2,
      this.size / 15,
      0,
      2 * Math.PI
    );
    ctx.fill();
    if (this.getAttribute("wasd") !== null) {
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillStyle = "#84899b";
      ctx.fillText("w", this.size / 2, (this.size * 1) / 4);
      ctx.fillText("a", (this.size * 1) / 4, this.size / 2);
      ctx.fillText("s", this.size / 2, (this.size * 3) / 4);
      ctx.fillText("d", (this.size * 3) / 4, this.size / 2);
    }
  }
}

customElements.define("formant-joystick", Joystick);
