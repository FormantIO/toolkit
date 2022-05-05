import * as uuid from "uuid";
import { defined } from "../../../common/defined";
import { IMap } from "../../../data-sdk/src/model/IMap";
import { IUniverseData } from "../model/IUniverseData";
import { GridMap } from "../objects/GridMap";
import { LayerSuggestion } from "./LayerRegistry";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class GridMapLayer extends UniverseLayerContent {
  static layerTypeId: string = "grid_map";

  static commonName = "Grid Map";

  static description =
    "A grid map to represent a set of locations and data intensity.";

  static usesData = true;

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
              layerType: GridMapLayer.layerTypeId,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  map = new GridMap();

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    defined(this.universeData).subscribeToMap(
      defined(this.layerContext),
      defined(dataSource),
      this.onData
    );
    this.add(this.map);
  }

  onData = (data: IMap) => {
    this.map.map = data;
  };
}
