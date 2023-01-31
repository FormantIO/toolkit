import {
  Authentication,
  Fleet,
  IStreamData,
  IStreamTypeMap,
} from "@formant/data-sdk";

const MINUTE_IN_MILLISECONDS = 60000;

export const queryValues = async (
  deviceId: string
): Promise<IStreamData<keyof IStreamTypeMap>[] | undefined> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const streamsValues = await Fleet.queryTelemetry({
        start: new Date(Date.now() - MINUTE_IN_MILLISECONDS).toISOString(),
        end: new Date(Date.now()).toISOString(),
        deviceIds: [deviceId],
      });

      return streamsValues;
    }
  } catch (e) {
    throw e;
  }
};

const filterOutUndesireStreams = (
  currentStreams: IStreamData<keyof IStreamTypeMap>[],
  streamNames: string[]
) => currentStreams.filter((_) => streamNames.includes(_.name));

export const reduceCurrentValues = (
  currentStreams: IStreamData<keyof IStreamTypeMap>[],
  streamNames: string[]
) => {
  return currentStreams.reduce<any>((prev, current) => {
    if (streamNames.includes(current.name)) {
      const latestValue = current.points.at(-1)!;
      const streamName = current.name;

      if (current.type === "bitset") {
        const bitSet: [number, { keys: string[]; values: boolean[] }] =
          current.points.at(-1)! as any;

        const bits = bitSet[1].keys.reduce<any>((prevBit, currentBit, idx) => {
          prevBit[currentBit] = bitSet[1].values[idx];
          return prevBit;
        }, {});
        return { ...bits, ...prev };
      }
      prev[streamName] = latestValue[1];
      return prev;
    }
    return prev;
  }, {});
};

export const updateTable = async (
  deviceId: string,
  setter: (_: any) => void,
  configurationStreams: string[]
) => {
  while (getShowConfig() === "false") {
    const streams = await queryValues(deviceId);
    if (!streams) return;
    const formatedStreams = reduceCurrentValues(streams, configurationStreams);
    //TODO: Handle when stream is configured but not being publish to (undefined)
    await timeout(1000);
    setter(formatedStreams);
  }
};

export const timeout = (ms: number) => {
  return new Promise((resolve) => setTimeout(resolve, ms));
};

const getShowConfig = () => localStorage.getItem("show");
