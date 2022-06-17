import {
  BufferAttribute,
  BufferGeometry,
  Points,
  PointsMaterial,
  TextureLoader,
} from "three";
import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IPcd } from "../objects/pcd";
import { IUniverseData } from "../model/IUniverseData";
import { UniverseLayer } from "./UniverseLayer";
import { LayerSuggestion } from "./LayerRegistry";

export class PointCloudLayer extends UniverseLayer {
  static layerTypeId: string = "point_cloud";

  static commonName = "Point Cloud";

  static description = "A point cloud to represent a set of points.";

  static usesData = true;

  points!: Points;

  static fields = {
    pointSize: {
      name: "Point Size",
      description: "Size of points",
      placeholder: "0.01",
      value: 0.01,
      type: "number",
      location: ["create"],
    },
    pointColor: {
      name: "Point Color",
      description: "Color of points",
      placeholder: "",
      value: 0xffffff,
      type: "number",
      location: ["create"],
    },
    pointTexture: {
      name: "Point Texture",
      description: "Texture of points",
      placeholder: "",
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
      (await universeData.getTeleopRosStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.topicType === "sensor_msgs/PointCloud2") {
            dataLayers.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "realtime",
                  rosTopicName: stream.topicName,
                  rosTopicType: stream.topicType,
                },
              ],
              layerType: PointCloudLayer.layerTypeId,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  init() {
    const geom = new BufferGeometry();
    const MAX_POINTS = 350000;
    geom.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(MAX_POINTS * 3), 3)
    );
    geom.setDrawRange(0, 0);
    const texture = this.getFieldText("pointTexture");
    const color = this.getFieldNumber("pointColor") || 0xbac4e2;
    const size = this.getFieldNumber("pointSize") || 0.1;
    this.points = new Points(
      geom,
      new PointsMaterial({
        color,
        size,
        vertexColors: false,
        map: texture ? new TextureLoader().load(texture) : undefined,
        transparent: true,
        depthWrite: false,
      })
    );
    this.points.frustumCulled = false;
    const ninetyDegrees = Math.PI / 2;
    this.points.rotation.set(-ninetyDegrees, 0, 0);
    this.add(this.points);

    defined(this.universeData).subscribeToPointCloud(
      defined(this.getLayerContext()).deviceId,
      defined(this.layerDataSources)[0],
      (d) => this.onData(d)
    );
  }

  onData = (pcd: IPcd) => {
    this.points.geometry.setDrawRange(
      0,
      pcd.positions ? pcd.positions.length / 3 : 0
    );
    const points = Array.from(pcd.positions || []);
    const positionAttr = this.points.geometry.attributes
      .position as BufferAttribute;
    positionAttr.set(points, 0);
    positionAttr.needsUpdate = true;
  };
}

/*  const pcd =
      data.type === "telemetry_point_cloud"
        ? parse(
            await fetch(data.pointCloud.url, { mode: "cors" }).then((r) =>
              r.arrayBuffer()
            )
          )
        : loadFromBase64(data.pointCloud.data); */
