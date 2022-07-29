import { ServiceParameters, ServiceParameterSetter } from "./ServiceParameters";
type FieldType =
  | "string"
  | "integer"
  | "number"
  | "array"
  | "object"
  | "boolean";

interface JsonBaseSchema<T extends FieldType> {
  title: string;
  type: T;
  default?: string;
}

export interface JsonObjectSchema extends JsonBaseSchema<"object"> {
  properties: { [key: string]: JsonSchema };
}

export interface JsonArraySchema extends JsonBaseSchema<"array"> {
  items: {
    type: "string" | "integer" | "number" | "boolean";
  };
}

export type JsonStringSchema = JsonBaseSchema<"string">;
export type JsonBooleanSchema = JsonBaseSchema<"boolean">;
export type JsonIntegerSchema = JsonBaseSchema<"integer">;
export type JsonNumberSchema = JsonBaseSchema<"number">;

export type JsonSchema =
  | JsonObjectSchema
  | JsonStringSchema
  | JsonArraySchema
  | JsonBooleanSchema
  | JsonIntegerSchema
  | JsonNumberSchema;

export interface IInputProps<TSchema extends JsonSchema = JsonSchema> {
  schema: TSchema;
  params: ServiceParameters;
  path: string[];
  setParams: ServiceParameterSetter;
}
