import { useEffect, useState } from "react";
import {
  App,
  IStreamData,
  IBitset,
  Authentication,
  Fleet,
} from "@formant/data-sdk";
import { ICurrentValues } from "types";
import { useFormant } from "@formant/ui-sdk";

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;

const reduceStreamValues = (currentValue: IStreamData[]) =>
  currentValue.reduce<any>((prev, currentStream) => {
    const latesPoint = currentStream.points.at(-1);
    if (!latesPoint) return prev;
    const value = latesPoint[1];
    const timeStamp = latesPoint[0];
    if (timeStamp < Date.now() - 10 * SECONDS) return prev;

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

export const useCurrentStreamsValues = (streams: string[]): ICurrentValues => {
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
      types: ["text", "numeric", "bitset"],
    });

    const reducedValues = reduceStreamValues(
      curentDeviceStreams as IStreamData[]
    );
    setCurrentValues(reducedValues);
  };

  const handleStreams = async () => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getCurrentDevice();
    App.addStreamListener(
      streams,
      ["text", "numeric", "bitset"],
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
          deviceStreams as IStreamData[]
        );
        setCurrentValues(reducedValues);
      }
    );
  };

  return currentValues;
};
