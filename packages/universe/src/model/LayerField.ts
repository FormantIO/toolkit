export type LayerFieldLocation = "create" | "edit";

export type LayerFieldType = "text" | "number" | "boolean";

export type LayerFieldTypeMap = {
  text: string;
  number: number;
  boolean: boolean;
};

type LayerFieldValue<T extends LayerFieldType = LayerFieldType> = {
  type: T;
  value?: LayerFieldTypeMap[T];
};

export interface LayerField<T extends LayerFieldType = LayerFieldType>
  extends LayerFieldValue<T> {
  name: string;
  location: LayerFieldLocation[];
  description: string;
  placeholder: LayerFieldTypeMap[T];
}

export type LayerFieldUnion = {
  [Type in keyof LayerFieldTypeMap]: LayerField<Type>;
}[keyof LayerFieldTypeMap];
type LayerFieldValuesUnion = {
  [Type in keyof LayerFieldTypeMap]: LayerFieldValue<Type>;
}[keyof LayerFieldTypeMap];

export type LayerFields = { [key in string]: LayerFieldUnion };
export type LayerFieldValues = { [key: string]: LayerFieldValuesUnion };

export function extractLayerFieldValues(
  layerFields: LayerFields
): LayerFieldValues {
  const values: LayerFieldValues = {};
  Object.entries(layerFields).forEach(([key, field]) => {
    values[key] = {
      type: field.type,
      value: field.value,
    } as LayerFieldValuesUnion;
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
