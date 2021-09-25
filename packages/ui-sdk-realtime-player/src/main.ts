import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
// @ts-ignore
import RealtimePlayerWorker from "../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

export class RealtimePlayer extends HTMLElement {
  drawer: H264BytestreamCanvasDrawer;
  constructor() {
    super();
    this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
  }
  connectedCallback() {
    this.style.background = "black";
    this.innerHTML = `<canvas style="height: 100%; width: 100%; object-fit: contain;"></canvas>`;
    this.drawer.start();
    this.drawer.setCanvas(this.querySelector("canvas") as HTMLCanvasElement);
  }

  drawVideoFrame(h264Frame: any) {
    this.drawer.receiveEncodedFrame(h264Frame);
  }
}

customElements.define("formant-realtime-player", RealtimePlayer);
