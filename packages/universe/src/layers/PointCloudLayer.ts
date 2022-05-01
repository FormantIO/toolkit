import { BufferAttribute, BufferGeometry, Points, PointsMaterial } from "three";
import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IRtcPointCloud } from "./IRtcPointCloud";
import { loadFromBase64 } from "./pcd";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";
import { LayerSuggestion } from "./LayerRegistry";

export class PointCloudLayer extends UniverseLayerContent {
  static id = "point_cloud";

  static commonName = "Point Cloud";

  static description = "A point cloud to represent a set of points.";

  static usesData = true;

  points: Points;

  static createDefault(
    universeData: IUniverseData,
    _deviceId: string,
    universeDataSources?: UniverseDataSource[]
  ): TransformLayer<PointCloudLayer> {
    return new TransformLayer(
      new PointCloudLayer(universeData, defined(universeDataSources)[0])
    );
  }

  static getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): LayerSuggestion[] {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      universeData.getTeleopRosStreams().forEach((stream) => {
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
            layerType: PointCloudLayer.id,
          });
        }
      });
    }
    return dataLayers;
  }

  constructor(
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource
  ) {
    super();
    defined(this.universeData).subscribeToDataSource(
      defined(this.dataSource),
      this.onData
    );

    const geom = new BufferGeometry();
    const MAX_POINTS = 10000;
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

  onData = (data: IRtcPointCloud) => {
    const pcd = loadFromBase64(data.data);
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
