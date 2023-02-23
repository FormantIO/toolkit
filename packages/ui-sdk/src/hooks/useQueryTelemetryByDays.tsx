import { Authentication, Fleet } from "@formant/data-sdk";
import useDevice from "./useDevice";
import { useState, useEffect } from "react";

type AggregateLevel = "year" | "month" | "week" | "day" | "hour" | "minute";

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

const useQueryTelemetryByDays = (
  types: DataType[],
  rangeInDays?: string | number | undefined,
  aggregate?: AggregateLevel
) => {
  const device = useDevice();

  const [values, setvalues] = useState<any[]>();
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    setQuery(rangeInDays);
  }, [rangeInDays]);

  const setQuery = async (days: string | number | undefined) => {
    if (device === undefined) return;
    if (days === undefined) return;
    const start = getStartISODate(days);
    if (start === undefined) return;
    const end = new Date().toISOString();
    setIsLoading(true);
    if (await Authentication.waitTilAuthenticated()) {
      let telemetry = await Fleet.queryTelemetry({
        start,
        end,
        aggregate,
        types: types,
        deviceIds: [device.id],
      });
      setvalues(telemetry);
      setIsLoading(false);
    }
  };

  return [values, setQuery, isLoading] as const;
};

const getStartISODate = (range: number | string | undefined) => {
  if (typeof range === "string") range = parseInt(range);
  if (typeof range === "string" && isNaN(range)) return;
  if (range === undefined) range = 1;
  let daysInMilli = 86400000;
  daysInMilli = daysInMilli * range;
  let startDate = Date.now();
  startDate = startDate - daysInMilli;
  return new Date(startDate).toISOString();
};

export default useQueryTelemetryByDays;
