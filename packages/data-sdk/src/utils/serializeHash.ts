import pako from "pako";
import b64 from "base64-js";

const encoder = new TextEncoder();
const decoder = new TextDecoder();

export function serializeHash(value: any): string {
  const jsonValue = JSON.stringify(value);
  const encodedValue = encoder.encode(jsonValue);
  const compressedValue = pako.deflate(encodedValue);
  return b64.fromByteArray(compressedValue);
}

export function deserializeHash(value: string): any {
  const a = b64.toByteArray(value);
  const b = pako.inflate(a);
  const c = decoder.decode(b);
  return JSON.parse(c);
}
