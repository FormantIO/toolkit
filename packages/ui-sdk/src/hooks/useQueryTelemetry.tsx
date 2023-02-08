import { Authentication, Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import useDevice from "./useDevice";

type AggregateLevel =
  | "year"
  | "month"
  | "week"
  | "day"
  | "hour"
  | "minute"
  | "second";

type DataType =
  | "bitset"
  | "localization"
  | "point cloud"
  | "location"
  | "file"
  | "health"
  | "transform tree"
  | "battery"
  | "video"
  | "numeric set"
  | "json"
  | "image"
  | "numeric"
  | "text";

const useQueryTelemetry = (
  types: DataType[],
  start: string,
  end: string,
  aggregate?: AggregateLevel
) => {
  const device = useDevice();
  const [queryTelemetry, setQueryTelemetry] = useState<any[]>();

  useEffect(() => {
    getQueryTelemetry();
  }, [device]);

  const getQueryTelemetry = async () => {
    if (device === undefined) return;
    let telemetry: any[] = [];
    if (await Authentication.waitTilAuthenticated()) {
      telemetry = await Fleet.queryTelemetry({
        start,
        end,
        aggregate,
        types: types,
        deviceIds: [device.id],
      });
    }
    setQueryTelemetry(telemetry);
  };
  return queryTelemetry;
};

export default useQueryTelemetry;
