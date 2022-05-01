import { Object3D, PerspectiveCamera } from "three";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";

export interface TextLayerFieldValue {
  type: string;
  value?: string;
}

export interface TextLayerField extends TextLayerFieldValue {
  name: string;
  description: string;
  placeholder: string;
}

export type LayerField = TextLayerField;
export type LayerFieldValue = TextLayerFieldValue;

export type LayerFields = { [key in string]: LayerField };
export type LayerFieldValues = { [key: string]: TextLayerFieldValue };

export function extractLayerFieldValues(
  layerFields: LayerFields
): LayerFieldValues {
  const values: LayerFieldValues = {};
  for (const [key, field] of Object.entries(layerFields)) {
    values[key] = {
      type: field.type,
      value: field.value,
    };
  }
  return values;
}

export function injectLayerFieldValues(
  layerFields: LayerFields,
  layerFieldValues: LayerFieldValues
): void {
  Object.entries(layerFieldValues).forEach(([key, value]) => {
    layerFields[key].value = value.value;
  });
}

export abstract class UniverseLayerContent extends Object3D {
  static id: string;

  static commonName: string;

  static description: string;

  static usesData: boolean;

  static fields?: LayerFields;

  static getLayerSuggestions(
    _data: IUniverseData,
    _deviceContext?: string
  ): LayerSuggestion[] {
    return [];
  }

  static createDefault(
    _universeData: IUniverseData,
    _deviceId?: string,
    _universeDataSources?: UniverseDataSource[],
    _fields?: LayerFields,
    _camera?: () => PerspectiveCamera
  ): TransformLayer<UniverseLayerContent> {
    throw new Error("Method not implemented.");
  }
}
