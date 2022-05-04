export type LayerFieldLocation = "create" | "edit";
export interface TextLayerFieldValue {
  type: string;
  location: string[];
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
  Object.entries(layerFields).forEach(([key, field]) => {
    values[key] = {
      type: field.type,
      value: field.value,
      location: field.location,
    };
  });
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
