import { useCallback, useEffect, useState } from "react";
import {
  App,
  Authentication,
  ModuleData,
  IStreamData,
  IStreamTypeMap,
  Device,
  Fleet,
} from "@formant/data-sdk";
import { useDevice, useFormant } from "@formant/ui-sdk";

interface data {
  agentId: string;
  deviceId: string;
  name: string;
  tags: { [key: string]: string };
  points: [number, any][];
  type: string;
}

type IStream = [string, Stream];

interface Stream {
  loading: boolean;
  tooMuchData: boolean;
  data: data[];
  type: string;
}

type Streams = { [key: string]: any };

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;

const useModuleDataListener = () => {
  const context = useFormant();
  const config = context.configuration as {
    fullScreenMode: boolean;
    stream: string;
  };
  const [streams, setStreams] = useState<Streams | null>({});
  useEffect(() => {
    if (!!!config) return;
    if (!config.fullScreenMode) {
      handleData();
      return;
    }
    const dataListener = setInterval(() => {
      handleFullScreenMode();
    }, 2000);
    return () => {
      clearInterval(dataListener);
    };
  }, [config]);

  const handleFullScreenMode = async () => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getCurrentDevice();

    const curentDeviceStream = await Fleet.queryTelemetry({
      deviceIds: [device.id],
      start: new Date(Date.now() - MINUTES * 2).toISOString(),
      end: new Date(Date.now()).toISOString(),
      names: [config.stream],
      types: ["json"],
    });
    const latestPoint = curentDeviceStream[0].points.at(-1);
    if (latestPoint) {
      const streams = { Diagnostics: latestPoint[1] };
      setStreams({ ...streams });
    }
  };

  const handleData = useCallback(async () => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getCurrentDevice();

    App.addStreamListener([config.stream], ["json"], (s) => {
      if (!s) return;
      const curentDeviceStream = (
        s as IStreamData<keyof IStreamTypeMap>[]
      ).filter((_) => _.deviceId === device.id);
      const { name } = curentDeviceStream[0];
      const latestPoint = curentDeviceStream[0].points.at(-1);
      if (latestPoint) {
        const streams = { [name]: latestPoint[1] };
        setStreams({ ...streams });
      }
    });
  }, [config]);

  return streams;
};

export default useModuleDataListener;
