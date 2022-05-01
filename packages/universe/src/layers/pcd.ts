// @ts-ignore-next-line
import * as lzfjs from "lzfjs";
import { range } from "../../../common/range";

export type Field = "x" | "y" | "z" | "rgb" | "rgba" | "intensity";
export type Type = "i" | "u" | "f";
export type Data = "ascii" | "binary" | "binary_compressed";

export interface IPcdHeader {
  version: string;
  fields: Field[];
  size: number[];
  type: Type[];
  count: number[];
  height: number;
  width: number;
  points: number;
  data: Data;
}

interface IOffsets {
  x?: number;
  y?: number;
  z?: number;
  rgb?: number;
  rgba?: number;
  intensity?: number;
}

export interface IPcd {
  header: IPcdHeader;
  positions?: Float32Array;
  colors?: Float32Array;
  intensity?: Float32Array;
}

const littleEndian = true;

function base64ToArrayBuffer(b: string) {
  const binaryString = window.atob(b);
  const len = binaryString.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i += 1) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return bytes.buffer;
}

function extractHeader(buffer: ArrayBuffer): {
  header: string;
  body: ArrayBuffer;
} {
  const chars = new Uint8Array(buffer);
  let header = "";

  let i = 0;
  for (
    ;
    i < chars.length && header.search(/[\r\n]DATA\s(\S*)\s/i) === -1;
    i += 1
  ) {
    header += String.fromCharCode(chars[i]);
  }
  return {
    body: buffer.slice(i),
    header: header.replace(/#.*/gi, ""),
  };
}

function decompress(data: ArrayBuffer): ArrayBufferLike {
  const sizes = new Uint32Array(data, 0, 2);
  const compressedSize = sizes[0];
  if (compressedSize === 0) {
    return new ArrayBuffer(0);
  }
  return lzfjs.decompress(new Uint8Array(data, 8, compressedSize)).buffer;
}

function parseHeader(buffer: ArrayBuffer): {
  header: IPcdHeader;
  body: ArrayBuffer;
} {
  const { header, body } = extractHeader(buffer);

  const versionMatch = /VERSION (.*)/i.exec(header);
  if (versionMatch === null) {
    throw new Error(`Missing version. Header ${header}`);
  }
  const version = versionMatch[1];

  const fieldsMatch = /FIELDS (.*)/i.exec(header);
  if (!fieldsMatch) {
    throw new Error("Missing fields");
  }
  const fields = fieldsMatch[1].split(" ") as Field[];

  const sizeMatch = /SIZE (.*)/i.exec(header);
  if (!sizeMatch) {
    throw new Error("Missing size");
  }
  const size = sizeMatch[1].split(" ").map((_) => parseInt(_, 10));

  const typeMatch = /TYPE (.*)/i.exec(header);
  if (!typeMatch) {
    throw new Error("Missing type");
  }
  const type = typeMatch[1].split(" ") as Type[];

  const countMatch = /COUNT (.*)/i.exec(header);
  let optionalCount: number[] | undefined;
  if (countMatch) {
    optionalCount = countMatch[1].split(" ").map((_) => parseInt(_, 10));
  }
  const count = optionalCount || fields.map((_) => 1);

  const widthMatch = /WIDTH (.*)/i.exec(header);
  if (!widthMatch) {
    throw new Error("Missing width");
  }
  const width = parseInt(widthMatch[1], 10);

  const heightMatch = /HEIGHT (.*)/i.exec(header);
  if (!heightMatch) {
    throw new Error("Missing height");
  }
  const height = parseInt(heightMatch[1], 10);

  const pointsMatch = /POINTS (.*)/i.exec(header);
  let optionalPoints: number | undefined;
  if (pointsMatch) {
    optionalPoints = parseInt(pointsMatch[1], 10);
  }
  const points = optionalPoints || width * height;

  const dataMatch = /DATA (.*)/i.exec(header);
  if (!dataMatch) {
    throw new Error("Missing data");
  }
  const data = dataMatch[1] as Data;

  return {
    body,
    header: {
      count,
      data,
      fields,
      height,
      points,
      size,
      type,
      version,
      width,
    },
  };
}

function calculateOffsets(header: IPcdHeader): {
  offsets: IOffsets;
  size: number;
} {
  const empty: IOffsets = {};
  return header.fields.reduce(
    ({ offsets, size }, field, i) => {
      let newSize = size;
      if (field === "x") {
        offsets.x = newSize;
      }
      if (field === "y") {
        offsets.y = newSize;
      }
      if (field === "z") {
        offsets.z = newSize;
      }
      if (field === "rgb") {
        offsets.rgb = newSize;
      }
      if (field === "rgba") {
        offsets.rgba = newSize;
      }
      if (field === "intensity") {
        offsets.intensity = newSize;
      }
      if (header.data === "ascii") {
        newSize += 1;
      } else if (header.data === "binary") {
        newSize += header.size[i] * header.count[i];
      } else if (header.data === "binary_compressed") {
        newSize += header.size[i] * header.count[i] * header.points;
      }
      return {
        offsets,
        size: newSize,
      };
    },
    {
      offsets: empty,
      size: 0,
    }
  );
}

function parse(buffer: ArrayBuffer): IPcd {
  const { header, body } = parseHeader(buffer);

  const { offsets, size } = calculateOffsets(header);

  let positions: Float32Array | undefined;
  if (
    offsets.x !== undefined &&
    offsets.y !== undefined &&
    offsets.z !== undefined
  ) {
    positions = new Float32Array(header.points * 3);
  }

  let colors: Float32Array | undefined;
  if (offsets.rgb !== undefined || offsets.rgba !== undefined) {
    colors = new Float32Array(header.points * 4);
  }
  let intensity: Float32Array | undefined;
  if (offsets.intensity !== undefined) {
    intensity = new Float32Array(header.points);
  }

  if (header.data === "ascii") {
    const dataString: string = String.fromCharCode(...new Uint8Array(body));

    const lines = dataString.split("\n");
    lines.forEach((line, i) => {
      const column = line.split(" ");

      if (positions !== undefined) {
        positions[i * 3 + 0] = parseFloat(column[offsets.x || 0]);
        positions[i * 3 + 1] = parseFloat(column[offsets.y || 0]);
        positions[i * 3 + 2] = parseFloat(column[offsets.z || 0]);
      }

      if (colors !== undefined) {
        const intArray = new Int32Array([
          parseInt(column[offsets.rgb || offsets.rgba || 0], 10),
        ]);
        const view = new DataView(intArray.buffer, 0);
        colors[i * 3 + 0] = view.getUint8(2) / 255.0;
        colors[i * 3 + 1] = view.getUint8(1) / 255.0;
        colors[i * 3 + 2] = view.getUint8(0) / 255.0;
      }

      if (intensity !== undefined) {
        intensity[i] = parseFloat(column[offsets.intensity || 0]);
      }
    });
  } else if (header.data === "binary") {
    const view = new DataView(body);

    range(0, header.points).forEach((i) => {
      const row = size * i;

      if (positions !== undefined) {
        positions[i * 3 + 0] = view.getFloat32(
          row + (offsets.x || 0),
          littleEndian
        );
        positions[i * 3 + 1] = view.getFloat32(
          row + (offsets.y || 0),
          littleEndian
        );
        positions[i * 3 + 2] = view.getFloat32(
          row + (offsets.z || 0),
          littleEndian
        );
      }

      if (colors !== undefined) {
        const offset = row + (offsets.rgb || offsets.rgba || 0);
        if (offset + 2 <= view.byteLength) {
          colors[i * 4 + 0] = view.getUint8(offset + 2) / 255.0;
          colors[i * 4 + 1] = view.getUint8(offset + 1) / 255.0;
          colors[i * 4 + 2] = view.getUint8(offset + 0) / 255.0;
          colors[i * 4 + 3] = 1.0;
        }
      }

      if (intensity !== undefined) {
        intensity[i] = view.getFloat32(
          row + (offsets.intensity || 0),
          littleEndian
        );
      }
    });
  } else if (header.data === "binary_compressed") {
    const uncompressed = decompress(body);
    const view = new DataView(uncompressed);

    range(0, header.points).forEach((i) => {
      if (positions !== undefined) {
        positions[i * 3 + 0] = view.getFloat32(
          (offsets.x || 0) + i * 4,
          littleEndian
        );
        positions[i * 3 + 1] = view.getFloat32(
          (offsets.y || 0) + i * 4,
          littleEndian
        );
        positions[i * 3 + 2] = view.getFloat32(
          (offsets.z || 0) + i * 4,
          littleEndian
        );
      }

      if (colors !== undefined) {
        const offset = (offsets.rgb || offsets.rgba || 0) + i * 4;
        if (offset + 2 <= view.byteLength) {
          colors[i * 4 + 0] = view.getUint8(offset + 2) / 255.0;
          colors[i * 4 + 1] = view.getUint8(offset + 1) / 255.0;
          colors[i * 4 + 2] = view.getUint8(offset + 0) / 255.0;
          colors[i * 4 + 3] = 1.0;
        }
      }

      if (intensity !== undefined) {
        intensity[i] = view.getFloat32(
          (offsets.intensity || 0) + i * 4,
          littleEndian
        );
      }
    });
  }

  return {
    colors,
    header,
    intensity,
    positions,
  };
}

export async function loadFromUrl(path: string): Promise<IPcd> {
  const response = await fetch(path, { mode: "cors" });
  return parse(await response.arrayBuffer());
}

export function loadFromBase64(data: string): IPcd {
  return parse(base64ToArrayBuffer(data));
}
