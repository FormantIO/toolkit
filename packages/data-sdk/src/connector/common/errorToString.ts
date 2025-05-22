import { errorToObject } from "./errorToObject";
import { toStringSafe } from "./toStringSafe";

/**
 * Serialize an error object to a string, including any custom properties.
 */
export function errorToString(error: unknown): string {
  const { message, stack, meta } = errorToObject(error);

  return `${stack || message || ""}${
    meta && Object.keys(meta).length > 0 ? ` -- ${toStringSafe(meta)}` : ""
  }`;
}
