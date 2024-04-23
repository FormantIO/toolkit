import {
  IBitset,
  INumericSetEntry,
  IStreamData,
  StreamType,
  Timestamp,
} from "../../main";
import { range } from "./range";
import validator from "validator";

export interface IFlattenedPoint {
  streamName: string;
  streamType: StreamType;
  timestamp: Timestamp;
  value: any;
}

export function flattenPoints(streamData: IStreamData): IFlattenedPoint[] {
  // Some record types contain multiple values and are not exported as-is.
  // Multi-valued records are flattened into single-valued stream types.
  const { type, points } = streamData;
  switch (type) {
    case "bitset":
      // Bitset streams are flattened to numeric streams with the bitset
      // key appended to the stream name.
      return points.flatMap(([timestamp, value]) => {
        const { keys, values } = value as IBitset;
        if (keys.length !== values.length) {
          return [];
        }
        return range(0, keys.length).map<IFlattenedPoint>((_) => {
          return {
            streamName: `${streamData.name}.${keys[_]}`,
            streamType: "numeric",
            timestamp,
            value: values[_] ? 1 : 0,
          };
        });
      });
    case "numeric set":
      // Numeric sets are flattened
      return points.flatMap(([timestamp, value]) =>
        (value as INumericSetEntry[]).map<IFlattenedPoint>(
          ({ label, value: numericValue }) => ({
            streamName: `${streamData.name}.${label}`,
            streamType: "numeric",
            timestamp,
            value: numericValue,
          })
        )
      );
    case "json":
      // JSON values are emitted as JSON instead of a string
      return points.map(([timestamp, value]) => {
        // JSON value as json
        if (validator.isJSON(value as string)) {
          return {
            streamName: streamData.name,
            streamType: "json",
            timestamp,
            value: JSON.parse(value as string),
          };
        }
        // JSON value as url
        return {
          streamName: streamData.name,
          streamType: "json",
          timestamp,
          value,
        };
      });
    default:
      return points.map(([timestamp, value]) => {
        return {
          streamName: streamData.name,
          streamType: streamData.type,
          timestamp,
          value,
        };
      });
  }
}
