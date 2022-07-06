const fieldTypes = ["string", "integer", "number", "array", "object"] as const;

type FieldType = typeof fieldTypes[number];

export interface JsonSchema {
  title: string;
  type: FieldType;
  default: string;
  properties?: JsonSchema;
}
