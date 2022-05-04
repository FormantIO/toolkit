import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IMap } from "../../../data-sdk/src/model/IMap";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { GridMap } from "../objects/GridMap";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class GridMapLayer extends UniverseLayerContent {
  static id = "grid_map";

  static commonName = "Grid Map";

  static description =
    "A grid map to represent a set of locations and data intensity.";

  static usesData = true;

  static createDefault(
    layerId: string,
    universeData: IUniverseData,
    deviceId: string,
    universeDataSources?: UniverseDataSource[]
  ): TransformLayer<GridMapLayer> {
    return new TransformLayer(
      layerId,
      new GridMapLayer(
        layerId,
        deviceId,
        universeData,
        defined(universeDataSources)[0]
      ),
      deviceId
    );
  }

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      (await universeData.getTelemetryStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.disabled) {
            return;
          }
          if (
            stream.configuration.type === "ros-localization" &&
            stream.configuration.mapTopic
          ) {
            dataLayers.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "telemetry",
                  streamName: stream.name,
                  streamType: stream.configuration.type,
                },
              ],
              layerType: GridMapLayer.id,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  map = new GridMap();

  constructor(
    layerId?: string,
    deviceId?: string,
    private universeData?: IUniverseData,
    private dataSource?: UniverseDataSource
  ) {
    super(defined(layerId));
    defined(this.universeData).subscribeToMap(
      defined(deviceId),
      defined(this.dataSource),
      this.onData
    );
    this.add(this.map);
  }

  onData = (data: IMap) => {
    this.map.map = data;
  };
}
