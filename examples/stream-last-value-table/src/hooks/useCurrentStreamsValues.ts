import { useEffect, useState } from "react";
import {
  App,
  IStreamData,
  IBitset,
  Authentication,
  Fleet,
  INumericSetEntry,
} from "@formant/data-sdk";
import { ICurrentValues } from "types";
import { useFormant, useScrubberTime } from "@formant/ui-sdk";

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;

const reduceStreamValues = (
  currentValue: IStreamData[],
  scrubberTime?: number,
  teleopMode?: boolean
) => {
  const reducedValues = currentValue.reduce<any>((prev, currentStream) => {
    const latesPoint = currentStream.points.at(-1);
    if (!latesPoint) return prev;
    const value = latesPoint[1];
    const timeStamp = latesPoint[0];
    if (
      (!!scrubberTime && scrubberTime > Date.now() - 5 * SECONDS) ||
      teleopMode
    ) {
      if (timeStamp < Date.now() - 10 * SECONDS) return prev;
    }

    if (currentStream.type === "numeric set") {
      const keys = (value as INumericSetEntry[]).map((_) => ({
        [_.label]: `${Math.floor(_.value)}${_.unit}`,
      }));
      const numericSet = { [currentStream.name]: keys };
      return { ...prev, numericSet };
    }

    if (currentStream.type === "bitset") {
      const bits = (value as IBitset).keys.reduce<any>(
        (prevBit, currentBit, idx) => {
          prevBit[`${currentStream.name}/${currentBit}`] = (
            value as IBitset
          ).values[idx];
          return prevBit;
        },
        {}
      );
      return { ...prev, ...bits };
    }
    prev[currentStream.name] = value;
    return prev;
  }, {});

  return reducedValues;
};

export const useCurrentStreamsValues = (
  streams: string[],
  time: number
): ICurrentValues => {
  const context = useFormant();
  const config = context.configuration as { fullScreenMode: boolean };
  const [currentValues, setCurrentValues] = useState<ICurrentValues>({});

  useEffect(() => {
    if (!!!config) return;
    if (!config.fullScreenMode) {
      if (streams.length === 0) return;
      handleStreams();
      return;
    }
    const dataListener = setInterval(() => {
      handleFullScreenMode();
    }, 2000);
    return () => {
      clearInterval(dataListener);
    };
  }, [streams, config]);

  const handleFullScreenMode = async () => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getCurrentDevice();
    const curentDeviceStreams = await Fleet.queryTelemetry({
      deviceIds: [device.id],
      start: new Date(Date.now() - MINUTES * 2).toISOString(),
      end: new Date(Date.now()).toISOString(),
      names: streams,
      types: ["text", "numeric", "bitset", "numeric set"],
    });

    const reducedValues = reduceStreamValues(
      curentDeviceStreams as IStreamData[],
      undefined,
      true
    );
    setCurrentValues(reducedValues);
  };

  const handleStreams = async () => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getCurrentDevice();
    App.addStreamListener(
      streams,
      ["text", "numeric", "bitset", "numeric set"],
      (currentValue) => {
        if (!currentValue) return;
        if (currentValue.length === 0) {
          setCurrentValues({});
          return;
        }
        const deviceStreams = (currentValue as IStreamData[]).filter(
          (_) => _.deviceId === device.id
        );

        const reducedValues = reduceStreamValues(
          deviceStreams as IStreamData[],
          time
        );

        setCurrentValues(reducedValues);
      }
    );
  };

  return currentValues;
};
