import {
  BackSide,
  BoxGeometry,
  CanvasTexture,
  Mesh,
  MeshBasicMaterial,
  SphereGeometry,
} from "three";
import * as uuid from "uuid";
import { IUniverseData } from "../model/IUniverseData";
import { UniverseLayer } from "./UniverseLayer";
import { defined, definedAndNotNull } from "../../../common/defined";
import { LayerSuggestion } from "./LayerRegistry";

export class RealtimeVideoLayer extends UniverseLayer {
  static layerTypeId = "realtime_video";

  static commonName = "Realtime Video";

  static description = "A video representing a realtime camera.";

  static usesData = true;

  static fields = {
    videoShape: {
      name: "Shape",
      description: "The shape you'd like the video to be",
      placeholder: "sphere",
      value: "",
      type: "text",
      location: ["create"],
    },
  };

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

  ctx?: CanvasRenderingContext2D;

  texture?: CanvasTexture;

  mesh?: Mesh;

  init() {
    const dataSource = defined(this.layerDataSources)[0];

    defined(this.universeData).subscribeToRealtimeVideo(
      defined(this.layerContext),
      defined(dataSource),
      this.onData
    );
  }

  onData = (frame: HTMLCanvasElement) => {
    if (!this.mesh && frame.width > 0 && frame.height > 0) {
      const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
      canvas.width = frame.width;
      canvas.height = frame.height;
      const texture = new CanvasTexture(canvas);
      this.ctx = definedAndNotNull(canvas.getContext("2d"));
      this.texture = texture;

      const shapeField = (this.layerFields || {}).videoShape;
      const shape = shapeField.value;
      const ninetyDegrees = Math.PI / 2;
      if (shape === "sphere" || shape === "sphere_rotated") {
        const material = new MeshBasicMaterial({
          map: texture,
          side: BackSide,
        });
        const geometry = new SphereGeometry(0.3);
        const oneEightyDegrees = Math.PI;
        this.mesh = new Mesh(geometry, material);
        if (shape === "sphere_rotated") {
          this.mesh.rotation.set(oneEightyDegrees, 0, oneEightyDegrees);
        } else {
          this.mesh.rotation.set(ninetyDegrees, 0, 0);
        }
      } else {
        const material = new MeshBasicMaterial({
          map: texture,
        });
        const geometry = new BoxGeometry(1, 1, 0);
        this.mesh = new Mesh(geometry, material);
        this.mesh.rotation.set(ninetyDegrees, 0, 0);
      }
      this.add(this.mesh);
      if (shape !== "sphere") {
        this.mesh.scale.set(1, canvas.height / canvas.width, 0);
      }
      (this.mesh.material as any).map.needsUpdate = true;
      (this.mesh.material as any).needsUpdate = true;
    }
    if (this.ctx && this.texture) {
      this.ctx.drawImage(frame, 0, 0);
      this.texture.needsUpdate = true;
    }
  };
}

/*



import { IH264VideoFrame } from "../../../data-sdk/src/model/IH264VideoFrame";

import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
// @ts-ignore
// eslint-disable-next-line import/no-unresolved
import RealtimePlayerWorker from "../../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";


  this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
    canvas.width = 0;
    canvas.height = 0;
    this.drawer.setCanvas(canvas);
    this.drawer.start();

const drawer = defined(this.drawer);
    drawer.receiveEncodedFrame(frame);
    // we lazily create this canvas because threejs doesn't like size changes
    const canvas = defined(drawer.canvas);
    if (!this.mesh && canvas.width > 0 && canvas.height > 0) {
      const texture = new CanvasTexture(canvas);

      const shapeField = (this.layerFields || {}).videoShape;
      const shape = shapeField.value;
      if (shape === "sphere") {
        const material = new MeshBasicMaterial({
          map: texture,
          side: BackSide,
        });
        const geometry = new SphereGeometry(0.3);
        const oneEightyDegrees = Math.PI;
        this.mesh = new Mesh(geometry, material);
        this.mesh.rotation.set(oneEightyDegrees, 0, oneEightyDegrees);
      } else {
        const material = new MeshBasicMaterial({
          map: texture,
        });
        const geometry = new BoxGeometry(1, 1, 0);
        const ninetyDegrees = Math.PI / 2;
        this.mesh = new Mesh(geometry, material);
        this.mesh.rotation.set(ninetyDegrees, 0, 0);
      }
      this.add(this.mesh);
      if (shape !== "sphere") {
        this.mesh.scale.set(1, canvas.height / canvas.width, 0);
      }
      (this.mesh.material as any).map.needsUpdate = true;
      (this.mesh.material as any).needsUpdate = true;
    } */
