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

export class VideoLayer extends UniverseLayer {
  static layerTypeId = "video";

  static commonName = "Video";

  static description = "A video representing a camera.";

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
              layerType: VideoLayer.layerTypeId,
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

    defined(this.universeData).subscribeToVideo(
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
        this.mesh = new Mesh(geometry, material);
        if (shape === "sphere_rotated") {
          this.mesh.rotation.set(ninetyDegrees, 0, ninetyDegrees);
        }
      } else {
        const material = new MeshBasicMaterial({
          map: texture,
        });
        const geometry = new BoxGeometry(1, 1, 0);
        this.mesh = new Mesh(geometry, material);
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
