import * as uuid from "uuid";
import { IUniverseData } from  "@formant/universe-core";
import { defined } from "../../../common/defined";
import { IGridMap } from "../main";
import { GridMap } from "../objects/GridMap";
import { LayerSuggestion } from "./LayerRegistry";
import { UniverseLayer } from "./UniverseLayer";

export class GridMapLayer extends UniverseLayer {
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
    defined(this.universeData).subscribeToGridMap(
      defined(this.getLayerContext()).deviceId,
      defined(dataSource),
      this.onData
    );
    this.add(this.map);
  }

  onData = (data: IGridMap) => {
    this.map.map = data;
  };
}
