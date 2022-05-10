import { BufferAttribute, BufferGeometry, Points, PointsMaterial } from "three";
import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IRtcPointCloud } from "../../../data-sdk/src/model/IRtcPointCloud";
import { loadFromBase64, parse } from "../objects/pcd";
import { IUniverseData } from "../model/IUniverseData";
import { UniverseLayer } from "./UniverseLayer";
import { LayerSuggestion } from "./LayerRegistry";
import { IPointCloud } from "../../../data-sdk/src/model/IPointCloud";

export class PointCloudLayer extends UniverseLayer {
  static layerTypeId: string = "point_cloud";

  static commonName = "Point Cloud";

  static description = "A point cloud to represent a set of points.";

  static usesData = true;

  points!: Points;

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
    defined(this.universeData).subscribeToPointCloud(
      defined(this.layerContext),
      defined(this.layerDataSources)[0],
      (d) => this.onData(d)
    );

    const geom = new BufferGeometry();
    const MAX_POINTS = 350000;
    geom.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(MAX_POINTS * 3), 3)
    );
    geom.setDrawRange(0, 0);
    this.points = new Points(
      geom,
      new PointsMaterial({ size: 0.01, vertexColors: false })
    );

    this.add(this.points);
  }

  onData = async (
    data:
      | { type: "telemetry_point_cloud"; pointCloud: IPointCloud }
      | { type: "rtc_point_cloud"; pointCloud: IRtcPointCloud }
  ) => {
    const pcd =
      data.type === "telemetry_point_cloud"
        ? parse(
            await fetch(data.pointCloud.url, { mode: "cors" }).then((r) =>
              r.arrayBuffer()
            )
          )
        : loadFromBase64(data.pointCloud.data);
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
