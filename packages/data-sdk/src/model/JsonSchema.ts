import { StreamType } from "./StreamType";

export type FieldType =
  | "string"
  | "integer"
  | "number"
  | "array"
  | "object"
  | "boolean";

export interface IJsonBaseSchema<T extends FieldType> {
  title?: string;
  description?: string;
  type: T;
  default?: string;
  required?: string[];
  $schema?: string;
  "$formant.visible.when"?: [string, "=", string];
}

export interface IPropertyObject {
  [key: string]: JsonSchema;
}

export interface IJsonObjectSchema extends IJsonBaseSchema<"object"> {
  properties?: IPropertyObject;
  "$formant.documentationUrl"?: string;
}

export interface IJsonArraySchema extends IJsonBaseSchema<"array"> {
  items: {
    type: "integer" | "number" | "string" | "object";
    properties: IPropertyObject;
    required?: string[];
    "$formant.itemName"?: string;
  };
}

export interface IJsonStringSchema extends IJsonBaseSchema<"string"> {
  enum?: string[];
  "$formant.streams.byType"?: StreamType;
  "$formant.placeholder"?: string;
  "$formant.groups"?: boolean;
  "$formant.date"?: boolean;
}
export type IJsonBooleanSchema = IJsonBaseSchema<"boolean">;
export type IJsonIntegerSchema = IJsonBaseSchema<"integer">;
export type IJsonNumberSchema = IJsonBaseSchema<"number">;

export type JsonSchema =
  | IJsonObjectSchema
  | IJsonStringSchema
  | IJsonArraySchema
  | IJsonBooleanSchema
  | IJsonIntegerSchema
  | IJsonNumberSchema;
