import {
  BoxGeometry,
  BufferGeometry,
  Color,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshBasicMaterial,
  SphereGeometry,
  Sprite,
  SpriteMaterial,
  Texture,
  Vector3,
} from "three";
import * as uuid from "uuid";
import { defined, definedAndNotNull } from "../../../common/defined";
import { IMarker3DArray } from "../../../data-sdk/src/model/IMarker3DArray";
import { IUniverseData } from "../model/IUniverseData";
import { GeometryWorld } from "../objects/GeometryWorld";
import { LayerSuggestion } from "./LayerRegistry";
import { UniverseLayer } from "./UniverseLayer";

export class GeometryLayer extends UniverseLayer {
  static layerTypeId: string = "geometry";

  static commonName = "Geometry";

  static description = "Geometry driven from a ROS stream.";

  static usesData = true;

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      (await universeData.getTeleopRosStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.topicType === "visualization_msgs/MarkerArray") {
            dataLayers.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "realtime",
                  rosTopicName: stream.topicName,
                  rosTopicType: stream.topicType,
                },
              ],
              layerType: GeometryLayer.layerTypeId,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  world: GeometryWorld = new GeometryWorld();

  private worldGeometry: Map<string, Mesh | Line | Sprite> = new Map();

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    defined(this.universeData).subscribeToGeometry(
      defined(this.getLayerContext()).deviceId,
      defined(dataSource),
      this.onData
    );
  }

  onData = (markerArray: IMarker3DArray) => {
    this.world.processMarkers(markerArray);
    const geometry = this.world.getAllGeometry();

    geometry.forEach((g) => {
      if (g.dirty) {
        const mesh = this.worldGeometry.get(g.id);
        if (g.type === "line_list") {
          if (!mesh) {
            const material = new LineBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });

            const meshGeometry = new BufferGeometry().setFromPoints(
              g.points as Vector3[]
            );
            const lines = new Line(meshGeometry, material);
            lines.position.set(g.position.x, g.position.y, g.position.z);
            lines.scale.set(g.scale.x, g.scale.y, g.scale.z);
            lines.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);

            this.add(lines);
            this.worldGeometry.set(g.id, lines);
          } else {
            mesh.geometry.setFromPoints(g.points as Vector3[]);
            mesh.position.set(g.position.x, g.position.y, g.position.z);
            mesh.scale.set(g.scale.x, g.scale.y, g.scale.z);
            mesh.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);
            mesh.material = new LineBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });
          }
        } else if (g.type === "text") {
          const fontface = "Arial";
          const fontsize = 30;
          const message = g.text;
          const font = `${fontsize}px ${fontface}`;

          const canvas = document.createElement("canvas");
          const context = definedAndNotNull(canvas.getContext("2d"));

          // get size data (height depends only on font size)
          context.font = font;
          const metrics = context.measureText(message);
          const textWidth = metrics.width;
          const textHeight = fontsize;
          canvas.width = textWidth;
          canvas.height = textHeight;
          context.fillStyle = "#2d3855";
          context.fillRect(0, 0, textWidth, textHeight);

          // background color
          context.font = font;
          context.fillStyle = "#bac4e2";
          context.fillText(message, 0, fontsize);

          // canvas contents will be used for a texture
          const texture = new Texture(canvas);
          texture.needsUpdate = true;

          const spriteMaterial = new SpriteMaterial({
            map: texture,
          });

          if (!mesh) {
            const sprite = new Sprite(spriteMaterial);
            // make things less blurrier
            const pixelScale = 4;
            // scale sprite so it isn't stretched
            sprite.scale.set(
              1 / pixelScale,
              textHeight / textWidth / pixelScale,
              1.0 / pixelScale
            );
            this.add(sprite);
            this.worldGeometry.set(g.id, sprite);
          } else {
            mesh.material = spriteMaterial;
          }
        } else if (g.type === "sphere" || g.type === "cube") {
          if (!mesh) {
            const meshGeometry =
              g.type === "sphere"
                ? new SphereGeometry(1, 32, 16)
                : new BoxGeometry(1, 1, 1);
            const material = new MeshBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });
            const sphere = new Mesh(meshGeometry, material);
            sphere.position.set(g.position.x, g.position.y, g.position.z);
            sphere.scale.set(g.scale.x, g.scale.y, g.scale.z);
            sphere.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);

            this.add(sphere);
            this.worldGeometry.set(g.id, sphere);
          } else {
            mesh.position.set(g.position.x, g.position.y, g.position.z);
            mesh.scale.set(g.scale.x, g.scale.y, g.scale.z);
            mesh.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);
            mesh.material = new MeshBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });
          }
        } else if (g.type === "arrow") {
          if (!mesh) {
            const material = new LineBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });

            const meshGeometry = new BufferGeometry().setFromPoints(
              g.points as Vector3[]
            );
            const lines = new Line(meshGeometry, material);
            lines.position.set(g.position.x, g.position.y, g.position.z);
            lines.scale.set(g.scale.x, g.scale.y, g.scale.z);
            lines.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);

            this.add(lines);
            this.worldGeometry.set(g.id, lines);
          } else {
            mesh.geometry.setFromPoints(g.points as Vector3[]);
            mesh.position.set(g.position.x, g.position.y, g.position.z);
            mesh.scale.set(g.scale.x, g.scale.y, g.scale.z);
            mesh.rotation.set(g.rotation.x, g.rotation.y, g.rotation.z);
            mesh.material = new LineBasicMaterial({
              color: new Color(g.color.r, g.color.g, g.color.b),
              opacity: g.color.a,
            });
          }
        }
        g.dirty = false;
      }
    });

    const oldGeoIds = [...this.worldGeometry.keys()];
    const newGeoIds = geometry.map((g) => g.id);
    const toRemove = oldGeoIds.filter((id) => !newGeoIds.includes(id));
    toRemove.forEach((id) => {
      this.remove(defined(this.worldGeometry.get(id)));
      this.worldGeometry.delete(id);
    });
  };
}
