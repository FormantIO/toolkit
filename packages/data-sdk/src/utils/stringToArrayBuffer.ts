import { decode } from "base-64";

export function stringToArrayBuffer(input: string): Uint8Array {
  return Uint8Array.from(decode(input), (_) => _.charCodeAt(0));
}
