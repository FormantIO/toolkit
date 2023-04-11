import { Buffer } from "buffer";
import { deflateSync, inflateSync } from "zlib";

export function serializeHash(value: any): string {
  return deflateSync(Buffer.from(JSON.stringify(value), "utf8")).toString(
    "base64"
  );
}

export function deserializeHash(value: string): any {
  return JSON.parse(inflateSync(Buffer.from(value, "base64")).toString("utf8"));
}
