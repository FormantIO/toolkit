import { JsonObjectSchema } from "./JsonSchemaForm/types";
import { ServiceParameters } from "./ServiceParameters";

export const getDefaultParams = (
  service: JsonObjectSchema
): ServiceParameters =>
  Object.keys(service.properties).reduce((sum, key) => {
    const prop = service.properties[key];
    switch (prop.type) {
      case "object":
        return { ...sum, [key]: getDefaultParams(prop) };
      case "string":
      case "number":
      case "integer":
        return prop.default ? { ...sum, [key]: prop["default"] } : sum;
      case "boolean": {
        return prop.default
          ? { ...sum, [key]: /^true$/.test(prop["default"]) }
          : sum;
      }
      case "array":
        [""];
        return sum;
      default:
        return sum;
    }
  }, {});
