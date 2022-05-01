import { BoxGeometry, CanvasTexture, Mesh, MeshBasicMaterial } from "three";
import * as uuid from "uuid";
import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";
import { defined } from "../../../common/defined";
import { IH264VideoFrame } from "../../../data-sdk/src/model/IH264VideoFrame";
import { LayerSuggestion } from "./LayerRegistry";
// @ts-ignore
import RealtimePlayerWorker from "../../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

export class HardwareVideoLayer extends UniverseLayerContent {
  static id = "hardware_video";

  static commonName = "Hardware Video";

  static description = "A video plane representing a camera.";

  static usesData = true;

  static createDefault(
    universeData: IUniverseData,
    _deviceId: string,
    universeDataSources?: UniverseDataSource[]
  ): TransformLayer<HardwareVideoLayer> {
    return new TransformLayer(
      new HardwareVideoLayer(universeData, defined(universeDataSources)[0])
    );
  }

  static getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): LayerSuggestion[] {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      universeData.getHardwareStreams().forEach((stream) => {
        if (stream.rtcStreamType === "h264-video-frame") {
          dataLayers.push({
            sources: [
              {
                id: uuid.v4(),
                sourceType: "hardware",
                rtcStreamName: stream.name,
              },
            ],
            layerType: HardwareVideoLayer.id,
          });
        }
      });
    }
    return dataLayers;
  }

  loaded: boolean = false;

  drawer: H264BytestreamCanvasDrawer;

  canvas: HTMLCanvasElement;

  mesh: Mesh;

  constructor(
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource
  ) {
    super();
    this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    defined(this.universeData).subscribeToVideo(
      defined(this.dataSource),
      this.onData
    );
    this.canvas = document.createElement("CANVAS") as HTMLCanvasElement;

    const texture = new CanvasTexture(this.canvas);

    const material = new MeshBasicMaterial({
      map: texture,
    });

    const geometry = new BoxGeometry(1, 1, 0);
    const ninetyDegrees = Math.PI / 2;
    this.mesh = new Mesh(geometry, material);
    this.mesh.rotation.set(ninetyDegrees, 0, 0);
    this.add(this.mesh);
    this.drawer.setCanvas(this.canvas);
    this.drawer.start();
  }

  onData = (h264Frame: IH264VideoFrame) => {
    this.drawer.receiveEncodedFrame(h264Frame);
    this.mesh.scale.set(1, this.canvas.height / this.canvas.width, 0);
    (this.mesh.material as any).map.needsUpdate = true;
    (this.mesh.material as any).needsUpdate = true;
  };
}
