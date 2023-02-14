import { Authentication, Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";

const useLatestTelemetry = (
  deviceIdOrDeviceIds?: string | string[],
  dependencies: any[] = []
) => {
  const [latestTelemetry, setLatestTelemetry] = useState<any>();

  useEffect(() => {
    getLatestTelemetry();
  }, dependencies);

  const getLatestTelemetry = async () => {
    let _telemetry = null;
    if (await Authentication.waitTilAuthenticated()) {
      if (typeof deviceIdOrDeviceIds === "string") {
        _telemetry = await Fleet.getLatestTelemetry([deviceIdOrDeviceIds]);
      }
      if (!!deviceIdOrDeviceIds && typeof deviceIdOrDeviceIds !== "string") {
        _telemetry = await Fleet.getLatestTelemetry(deviceIdOrDeviceIds);
      }
      if (!!!deviceIdOrDeviceIds) {
        const _currentDevice = await Fleet.getCurrentDevice();
        _telemetry = await _currentDevice.getLatestTelemetry();
      }
      setLatestTelemetry(_telemetry);
    }
  };
  return latestTelemetry;
};

export default useLatestTelemetry;
