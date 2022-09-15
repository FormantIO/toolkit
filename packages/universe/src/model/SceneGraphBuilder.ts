import * as uuid from "uuid";
import { UniverseDataSource } from "@formant/universe-core";
import { PositioningBuilder } from "./PositioningBuilder";
import { Positioning, SceneGraph, SceneGraphElement } from "./SceneGraph";
import { LayerRegistry } from "../layers/LayerRegistry";
import { UniverseLayer } from "../layers/UniverseLayer";

export class SceneBuilder {
  scene: SceneGraph = [];

  deviceId!: string;

  constructor(deviceId: string) {
    this.deviceId = deviceId;
  }

  addGround(
    groundLayerConfig: {
      positioning: Positioning;
      flat: boolean;
    } = { positioning: PositioningBuilder.fixed(0, 0, 0), flat: true }
  ) {
    this.scene.push({
      id: uuid.v4(),
      editing: false,
      type: "ground",
      name: "Ground",
      children: [],
      visible: true,
      position: groundLayerConfig.positioning,
      fieldValues: {
        flatAxes: {
          type: "boolean",
          value: groundLayerConfig.flat,
        },
      },
      data: {},
    });
    return this;
  }

  addLayer(layer: SceneGraphElement) {
    layer.deviceContext = this.deviceId;
    this.scene.push(layer);
    return this;
  }

  addUrdf(
    urdfLayerConfig: {
      positioning: Positioning;
      dataSources: UniverseDataSource[];
    } = { positioning: PositioningBuilder.fixed(0, 0, 0), dataSources: [] }
  ) {
    this.scene.push({
      id: uuid.v4(),
      editing: false,
      type: "device_visual_tf",
      name: "URDF",
      deviceContext: this.deviceId,
      children: [],
      visible: true,
      position: urdfLayerConfig.positioning,
      fieldValues: {},
      data: {},
      dataSources: urdfLayerConfig.dataSources,
    });
    return this;
  }

  addGridMap(
    gridMapLayerConfig: {
      positioning: Positioning;
      dataSources: UniverseDataSource[];
    } = { positioning: PositioningBuilder.fixed(0, 0, 0), dataSources: [] }
  ) {
    this.scene.push({
      id: uuid.v4(),
      editing: false,
      type: "grid_map",
      name: "Grid Map",
      deviceContext: this.deviceId,
      children: [],
      visible: true,
      position: gridMapLayerConfig.positioning,
      fieldValues: {},
      data: {},
      dataSources: gridMapLayerConfig.dataSources,
    });

    return this;
  }

  addPointCloud(
    pointCloudLayerConfig: {
      positioning: Positioning;
      dataSources: UniverseDataSource[];
    } = { positioning: PositioningBuilder.fixed(0, 0, 0), dataSources: [] }
  ) {
    this.scene.push({
      id: uuid.v4(),
      editing: false,
      type: "point_cloud",
      name: "PointCloud",
      deviceContext: this.deviceId,
      children: [],
      visible: true,
      position: pointCloudLayerConfig.positioning,
      fieldValues: {
        pointColor: {
          type: "number",
          value: 0xffffff,
        },
        pointSize: {
          type: "number",
          value: 5,
        },
        pointTexture: {
          type: "text",
          value:
            "https://formant-3d-models.s3.us-west-2.amazonaws.com/point.png",
        },
        pointAttenuate: {
          type: "boolean",
          value: false,
        },
      },
      data: {},
      dataSources: pointCloudLayerConfig.dataSources,
    });
    return this;
  }

  addCustomLayer(
    customLayer: typeof UniverseLayer,
    customLayerConfig: {
      positioning: Positioning;
      dataSources: UniverseDataSource[];
    } = { positioning: PositioningBuilder.fixed(0, 0, 0), dataSources: [] }
  ) {
    LayerRegistry.register(customLayer);

    this.scene.push({
      id: uuid.v4(),
      editing: false,
      deviceContext: this.deviceId,
      type: customLayer.layerTypeId,
      name: customLayer.commonName,
      children: [],
      visible: true,
      position: customLayerConfig.positioning,
      fieldValues: {},
      data: customLayerConfig.dataSources,
    });
    return this;
  }

  build() {
    return this.scene;
  }
}
