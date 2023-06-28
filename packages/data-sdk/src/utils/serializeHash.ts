import { inflate, deflate } from "pako";
import { fromByteArray, toByteArray } from "base64-js";

const encoder = new TextEncoder();
const decoder = new TextDecoder();

export function serializeHash(value: any): string {
  const jsonValue = JSON.stringify(value);
  const encodedValue = encoder.encode(jsonValue);
  const compressedValue = deflate(encodedValue);
  return fromByteArray(compressedValue);
}

export function deserializeHash(value: string): any {
  const a = toByteArray(value);
  const b = inflate(a);
  const c = decoder.decode(b);
  return JSON.parse(c);
}
