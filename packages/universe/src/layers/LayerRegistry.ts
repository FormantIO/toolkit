import { PerspectiveCamera } from "three";
import { UniverseDataSource, IUniverseData } from "../model/IUniverseData";
import { LayerType } from ".";
import { defined } from "../../../common/defined";
import { UniverseLayer } from "./UniverseLayer";
import { LayerFields, LayerFieldLocation } from "../model/LayerField";
import { TransformLayer } from "./TransformLayer";

export interface LayerSuggestion {
  sources: UniverseDataSource[];
  layerType: LayerType;
}

export class LayerRegistry {
  private static layers: Map<LayerType, any> = new Map();

  static register(layer: any) {
    LayerRegistry.layers.set(layer.layerTypeId, layer);
  }

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<{
    nonDataLayers: LayerType[];
    dataLayers: LayerSuggestion[];
  }> {
    let suggestionsDataLayers: LayerSuggestion[] = [];
    await Promise.all(
      Array.from(LayerRegistry.layers.values()).map(async (layer) => {
        if (layer.usesData) {
          const suggestions = await layer.getLayerSuggestions(
            universeData,
            deviceContext
          );
          suggestionsDataLayers = suggestionsDataLayers.concat(suggestions);
        }
      })
    );

    return {
      nonDataLayers: Array.from(LayerRegistry.layers.entries())
        .filter(([_, v]) => !v.usesData)
        .map(([k]) => k),
      dataLayers: suggestionsDataLayers,
    };
  }

  static getCommonName(type: LayerType): string {
    return defined(LayerRegistry.layers.get(type)).commonName;
  }

  static getDescription(type: LayerType): string {
    return defined(
      LayerRegistry.layers.get(type),
      `Could not find layer of type ${type}`
    ).description;
  }

  static getFields(
    type: LayerType,
    location?: LayerFieldLocation
  ): LayerFields {
    const fields = LayerRegistry.layers.get(type)?.fields;
    const copy = JSON.parse(JSON.stringify(fields || {}));
    if (location) {
      Object.keys(copy).forEach((key) => {
        if (!copy[key].location.includes(location)) {
          delete copy[key];
        }
      });
    }
    return copy;
  }

  static createDefaultLayer(
    layerId: string,
    nodeType: LayerType,
    universeData: IUniverseData,
    deviceId?: string,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
    getCurrentCamera?: () => PerspectiveCamera,
    getTransformLayer?: (id: string) => TransformLayer
  ) {
    const Layer = defined(LayerRegistry.layers.get(nodeType));
    return UniverseLayer.createDefault(
      new Layer(),
      layerId,
      universeData,
      deviceId,
      dataSources,
      fields,
      getCurrentCamera,
      getTransformLayer
    );
  }
}
