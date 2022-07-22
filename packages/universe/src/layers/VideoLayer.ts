import {
  BackSide,
  BufferAttribute,
  BufferGeometry,
  CanvasTexture,
  Group,
  Material,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PlaneGeometry,
  SphereGeometry,
} from "three";
import * as uuid from "uuid";
import { IUniverseData } from  "@formant/universe-core";
import { UniverseLayer } from "./UniverseLayer";
import { defined, definedAndNotNull } from "../../../common/defined";
import { LayerSuggestion } from "./LayerRegistry";

export class VideoLayer extends UniverseLayer {
  static layerTypeId = "video";

  static commonName = "Video";

  static description = "A video representing a camera.";

  static usesData = true;

  static fields = {
    shape: {
      name: "Shape",
      description: "The shape you'd like the video to be",
      placeholder: "sphere",
      value: "",
      type: "text" as const,
      location: ["create" as const],
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

  group?: Group;

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    defined(this.universeData).subscribeToVideo(
      defined(this.getLayerContext()).deviceId,
      defined(dataSource),
      this.onData
    );
  }

  onLayerPartsRequested(): {
    [key in string]: Material | Mesh | Group | Object3D | undefined;
  } {
    return {
      "video-mesh": this.mesh,
    };
  }

  onData = (frame: HTMLCanvasElement) => {
    const shape = this.getField(VideoLayer.fields.shape);
    if (shape === "stereo-side-by-side") {
      if (!this.group) {
        const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
        canvas.width = frame.width;
        canvas.height = frame.height;
        const texture = new CanvasTexture(canvas);
        this.ctx = definedAndNotNull(canvas.getContext("2d"));
        this.texture = texture;
        const bufferedGeometryPlaneLeft = new BufferGeometry();
        const hsize = 0.5;
        const vsize = 0.5;
        bufferedGeometryPlaneLeft.setAttribute(
          "position",
          new BufferAttribute(
            new Float32Array([
              -hsize,
              -vsize,
              0,
              hsize,
              -vsize,
              0,
              hsize,
              vsize,
              0,
              -hsize,
              vsize,
              0,
            ]),
            3
          )
        );
        bufferedGeometryPlaneLeft.setAttribute(
          "uv",
          new BufferAttribute(new Float32Array([0, 0, 0.5, 0, 0.5, 1, 0, 1]), 2)
        );
        bufferedGeometryPlaneLeft.setIndex([0, 1, 2, 0, 2, 3]);

        const group = new Group();
        const leftEyePlane = new Mesh(
          bufferedGeometryPlaneLeft,
          new MeshBasicMaterial({
            map: texture,
            side: BackSide,
          })
        );
        leftEyePlane.layers.set(0);
        group.add(leftEyePlane);

        const bufferedGeometryPlaneRight = new BufferGeometry();
        bufferedGeometryPlaneRight.setAttribute(
          "position",
          new BufferAttribute(
            new Float32Array([
              -hsize,
              -vsize,
              0,
              hsize,
              -vsize,
              0,
              hsize,
              vsize,
              0,
              -hsize,
              vsize,
              0,
            ]),
            3
          )
        );
        bufferedGeometryPlaneRight.setAttribute(
          "uv",
          new BufferAttribute(new Float32Array([0.5, 0, 1, 0, 1, 1, 0.5, 1]), 2)
        );
        bufferedGeometryPlaneRight.setIndex([0, 1, 2, 0, 2, 3]);

        const rightEyePlane = new Mesh(
          bufferedGeometryPlaneRight,
          new MeshBasicMaterial({
            map: texture,
            side: BackSide,
          })
        );
        rightEyePlane.layers.set(1);
        group.add(rightEyePlane);
        this.add(group);

        leftEyePlane.scale.set(1, canvas.height / (canvas.width / 2), 0);
        rightEyePlane.scale.set(1, canvas.height / (canvas.width / 2), 0);
        this.group = group;
      }
      if (this.ctx && this.texture) {
        this.ctx.drawImage(frame, 0, 0);
        this.texture.needsUpdate = true;
      }
    } else {
      if (!this.mesh && frame.width > 0 && frame.height > 0) {
        const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
        canvas.width = frame.width;
        canvas.height = frame.height;
        const texture = new CanvasTexture(canvas);
        this.ctx = definedAndNotNull(canvas.getContext("2d"));
        this.texture = texture;

        const ninetyDegrees = Math.PI / 2;
        const isRotated =
          shape === "sphere_rotated" || shape === "sphere_rotated_fullscreen";
        const isSphere =
          shape === "sphere" ||
          shape === "sphere_rotated" ||
          shape === "sphere_fullscreen" ||
          shape === "sphere_rotated_fullscreen";
        if (isSphere) {
          const isFullScreen =
            shape === "sphere_fullscreen" ||
            shape === "sphere_rotated_fullscreen";
          const material = new MeshBasicMaterial({
            map: texture,
            side: BackSide,
          });
          const size = isFullScreen ? 500 : 0.3;
          const geometry = new SphereGeometry(size);
          this.mesh = new Mesh(geometry, material);
          if (isRotated) {
            this.mesh.rotation.set(-ninetyDegrees * 2, 0, 0);
          } else {
            this.mesh.rotation.set(ninetyDegrees, 0, 0);
          }
          this.mesh.scale.set(-1, 1, 1);
          if (isFullScreen) {
            material.depthTest = false;
            material.depthWrite = false;
            material.transparent = true;
          }
        } else {
          const material = new MeshBasicMaterial({
            map: texture,
          });
          const geometry = new PlaneGeometry(1, 1);
          this.mesh = new Mesh(geometry, material);
          this.mesh.rotateX(ninetyDegrees);
          this.mesh.rotateY(ninetyDegrees);
        }
        this.add(this.mesh);
        if (!isSphere) {
          this.mesh.scale.set(1, canvas.height / canvas.width, 0);
        }
        (this.mesh.material as any).map.needsUpdate = true;
        (this.mesh.material as any).needsUpdate = true;
      }
      if (this.ctx && this.texture) {
        this.ctx.drawImage(frame, 0, 0);
        this.texture.needsUpdate = true;
      }
    }
  };
}
