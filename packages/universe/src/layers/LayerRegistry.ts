import { PerspectiveCamera } from 'three';
import { UniverseDataSource, IUniverseData } from '../IUniverseData';
import { LayerType } from '.';
import { defined } from '../../../common/defined';
import { LayerFields, UniverseLayerContent } from './UniverseLayerContent';

export interface LayerSuggestion {
  sources: UniverseDataSource[];
  layerType: LayerType;
}
export class LayerRegistry {
  private static layers: Map<LayerType, typeof UniverseLayerContent> =
    new Map();

  static register(layer: typeof UniverseLayerContent) {
    LayerRegistry.layers.set(layer.id, layer);
  }

  static getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string,
  ): {
    nonDataLayers: LayerType[];
    dataLayers: LayerSuggestion[];
  } {
    let suggestionsDataLayers: LayerSuggestion[] = [];
    LayerRegistry.layers.forEach((layer) => {
      if (layer.usesData) {
        const suggestions = layer.getLayerSuggestions(
          universeData,
          deviceContext,
        );
        suggestionsDataLayers = suggestionsDataLayers.concat(suggestions);
      }
    });

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
    return defined(LayerRegistry.layers.get(type)).description;
  }

  static getFields(type: LayerType): LayerFields {
    const fields = LayerRegistry.layers.get(type)?.fields;
    return JSON.parse(JSON.stringify(fields || {}));
  }

  static createDefaultLayer(
    nodeType: LayerType,
    universeData: IUniverseData,
    deviceId: string,
    universeDataSources?: UniverseDataSource[],
    fields?: LayerFields,
    getCurrentCamera?: () => PerspectiveCamera,
  ) {
    const layer = defined(LayerRegistry.layers.get(nodeType));
    return layer.createDefault(
      universeData,
      deviceId,
      universeDataSources,
      fields,
      getCurrentCamera,
    );
  }
}
