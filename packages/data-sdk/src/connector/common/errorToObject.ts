import { toStringSafe } from "./toStringSafe";

/**
 * Convert an error object to plain javascript object, including any custom properties.
 */
export function errorToObject(
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  error: any
) {
  if (!error.stack) {
    return { message: toStringSafe(error) };
  }

  const { name, message, stack, ...meta } = error;

  return {
    name,
    message: message !== undefined ? message : toStringSafe(error),
    stack,
    meta: meta && Object.keys(meta).length === 0 ? undefined : meta,
  };
}
