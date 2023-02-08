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
  required?: string[];
  $schema?: string;
  "$formant.visible.when"?: [string, "=", string];
}

export interface IJsonBaseSchemaWithDefault<T extends FieldType, U>
  extends IJsonBaseSchema<T> {
  default?: U;
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

export interface IJsonStringSchema
  extends IJsonBaseSchemaWithDefault<"string", string> {
  enum?: string[];
  "$formant.streams.byType"?: StreamType;
  "$formant.placeholder"?: string;
  "$formant.groups"?: boolean;
  "$formant.date"?: boolean;
}
export type IJsonBooleanSchema = IJsonBaseSchemaWithDefault<"boolean", boolean>;
export type IJsonIntegerSchema = IJsonBaseSchemaWithDefault<"integer", number>;
export type IJsonNumberSchema = IJsonBaseSchemaWithDefault<"number", number>;

export type JsonSchema =
  | IJsonObjectSchema
  | IJsonStringSchema
  | IJsonArraySchema
  | IJsonBooleanSchema
  | IJsonIntegerSchema
  | IJsonNumberSchema;
