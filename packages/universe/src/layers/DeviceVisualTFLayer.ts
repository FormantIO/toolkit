import { Vector2 } from "three";
import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { ITransformNode } from "../../../data-sdk/src/model/ITransformNode";
import { TransformTree } from "../objects/TransformTree";
import { ITransformTreeNode } from "../objects/transformTreeLoader";
import { IUniverseData } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { UniverseLayer } from "./UniverseLayer";

export class DeviceVisualTFLayer extends UniverseLayer {
  static layerTypeId: string = "device_visual_tf";

  static commonName = "Transform Tree";

  static description = "A transform tree to represent a robot.";

  static usesData = true;

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const suggestions: LayerSuggestion[] = [];

    if (deviceContext) {
      await Promise.all(
        (
          await universeData.getTelemetryStreams(deviceContext)
        ).map(async (stream) => {
          if (
            (await universeData.getTelemetryStreamType(
              deviceContext,
              stream.name
            )) === "transform tree"
          ) {
            suggestions.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "telemetry",
                  streamName: stream.name,
                  streamType: "transform tree",
                },
              ],
              layerType: DeviceVisualTFLayer.layerTypeId,
            });
          }
        })
      );
    }
    return suggestions;
  }

  transformTree: TransformTree | undefined;

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    this.transformTree = new TransformTree(this.getCurrentCamera());
    this.add(this.transformTree);
    defined(this.universeData).subscribeToTransformTree(
      defined(this.getLayerContext()).deviceId,
      defined(dataSource),
      this.onTransformTreeData
    );
  }

  onTransformTreeData = async (data: ITransformNode) => {
    let transformData: ITransformTreeNode | undefined;
    if (data.url) {
      const result = await fetch(data.url);
      transformData = (await result.json()) as ITransformTreeNode;
    }
    if (transformData && this.transformTree) {
      this.transformTree.nodes = [transformData];
      this.transformTree.resolution = new Vector2(600, 400);
      this.transformTree.update();
    }
  };
}
