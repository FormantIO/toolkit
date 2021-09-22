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
    this.innerHTML = "<canvas></canvas>";
    this.drawer.setCanvas(this.querySelector("canvas") as HTMLCanvasElement);
  }
}

customElements.define("formant-realtime-player", RealtimePlayer);
