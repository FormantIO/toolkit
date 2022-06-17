export type LayerFieldLocation = "create" | "edit";

export type LayerFieldType = string | number | boolean;

export type LayerFieldValue = {
  type: string;
  location: string[];
  value?: LayerFieldType;
};

export interface LayerField extends LayerFieldValue {
  name: string;
  description: string;
  placeholder: string;
}

export type LayerFields = { [key in string]: LayerField };
export type LayerFieldValues = { [key: string]: LayerFieldValue };

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
    if (!layerFields[key]) {
      throw new Error(`Invalid layer field key: ${key}`);
    }
    layerFields[key].value = value.value;
  });
}
