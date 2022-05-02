import { PerspectiveCamera, Vector2 } from "three";
import * as uuid from "uuid";
import { defined } from "../../../../common/defined";
import { ITransformNode } from "../../../../data-sdk/src/model/ITransformNode";
import { TransformTree } from "./TransformTree";
import { ITransformTreeNode } from "./transformTreeLoader";
import { IUniverseData, UniverseDataSource } from "../../IUniverseData";
import { LayerSuggestion } from "../LayerRegistry";
import { TransformLayer } from "../TransformLayer";
import { LayerFields, UniverseLayerContent } from "../UniverseLayerContent";

export class DeviceVisualLayerTF extends UniverseLayerContent {
  static id = "device_visual_tf";

  static commonName = "Transform Tree";

  static description = "A transform tree to represent a robot.";

  static usesData = true;

  static createDefault(
    universeData: IUniverseData,
    _deviceId: string,
    universeDataSources?: UniverseDataSource[],
    _fields?: LayerFields,
    getCurrentCamera?: () => PerspectiveCamera
  ): TransformLayer<DeviceVisualLayerTF> {
    return new TransformLayer(
      new DeviceVisualLayerTF(
        universeData,
        defined(universeDataSources)[0],
        getCurrentCamera
      )
    );
  }

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
              layerType: DeviceVisualLayerTF.id,
            });
          }
        })
      );
    }
    return suggestions;
  }

  transformTree: TransformTree | undefined;

  constructor(
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource,
    getCamera?: () => PerspectiveCamera
  ) {
    super();
    if (getCamera) {
      this.transformTree = new TransformTree(getCamera());
      this.add(this.transformTree);
      defined(this.universeData).subscribeToTransformTree(
        defined(this.dataSource),
        this.onTransformTreeData
      );
    }
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
