import { BoxGeometry, CanvasTexture, Mesh, MeshBasicMaterial } from "three";
import * as uuid from "uuid";
import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
import { IUniverseData } from "../model/IUniverseData";
import { UniverseLayer } from "./UniverseLayer";
import { defined } from "../../../common/defined";
import { IH264VideoFrame } from "../../../data-sdk/src/model/IH264VideoFrame";
import { LayerSuggestion } from "./LayerRegistry";
// @ts-ignore
// eslint-disable-next-line import/no-unresolved
import RealtimePlayerWorker from "../../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

export class RealtimeVideoLayer extends UniverseLayer {
  static layerTypeId = "rtc_video";

  static commonName = "Realtime Video";

  static description = "A video plane representing a realtime camera.";

  static usesData = true;

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      (await universeData.getHardwareStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.rtcStreamType === "h264-video-frame") {
            dataLayers.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "hardware",
                  rtcStreamName: stream.name,
                },
              ],
              layerType: RealtimeVideoLayer.layerTypeId,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  loaded: boolean = false;

  drawer!: H264BytestreamCanvasDrawer;

  mesh!: Mesh;

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    defined(this.universeData).subscribeToRealtimeVideo(
      defined(this.layerContext),
      defined(dataSource),
      this.onData
    );
    const canvas = document.createElement("CANVAS") as HTMLCanvasElement;

    this.drawer.setCanvas(canvas);
    this.drawer.start();
  }

  onData = (frame: IH264VideoFrame) => {
    // we lazily create this canvas because threejs doesn't like size changes
    const canvas = defined(this.drawer.canvas);
    const texture = new CanvasTexture(canvas);

    const material = new MeshBasicMaterial({
      map: texture,
    });

    const geometry = new BoxGeometry(1, 1, 0);
    const ninetyDegrees = Math.PI / 2;
    this.mesh = new Mesh(geometry, material);
    this.mesh.rotation.set(ninetyDegrees, 0, 0);
    this.add(this.mesh);
    this.drawer.receiveEncodedFrame(frame);
    this.mesh.scale.set(1, canvas.height / canvas.width, 0);
    (this.mesh.material as any).map.needsUpdate = true;
    (this.mesh.material as any).needsUpdate = true;
  };
}
