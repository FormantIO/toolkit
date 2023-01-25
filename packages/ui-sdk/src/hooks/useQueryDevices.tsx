import * as React from "react";
import { Fleet, Device } from "@formant/data-sdk";
import { authenticate } from "../utils/authenticate";
import { IDeviceQuery } from "@formant/data-sdk/dist/types/data-sdk/src/model/IDeviceQuery";

const getDevices = async (query: IDeviceQuery): Promise<Device[]> => {
  return await authenticate(async () => {
    const devices = await Fleet.queryDevices(query);
    return devices;
  });
};

const useQueryDevices = (
  query: IDeviceQuery,
  deps: (string | number | boolean)[] = []
) => {
  const [devices, setDevices] = React.useState<Device[]>([]);

  React.useEffect(() => {
    getDevices(query).then((_) => setDevices(_));
  }, deps);

  return devices;
};

export default useQueryDevices;
