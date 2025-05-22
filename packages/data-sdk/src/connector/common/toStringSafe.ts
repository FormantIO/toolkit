import stringify from "fast-json-stable-stringify";

/**
 * Safely stringify input.
 * Similar to `JSON.stringify` but
 * doesn't throw an error if input includes circular properties.
 *
 * Warning: output is not guaranteed to be a valid JSON string.
 */
export function toStringSafe(input: unknown): string {
  return stringify(input, { cycles: true });
}
