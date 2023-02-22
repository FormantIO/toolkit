import { useEffect, useState } from "react";
import { App, IStreamData, IStreamTypeMap, IBitset } from "@formant/data-sdk";
import { ICurrentValues } from "types";

const SECONDS = 1000;

export const useCurrentStreamsValues = (streams: string[]): ICurrentValues => {
  const [currentValues, setCurrentValues] = useState<ICurrentValues>({});

  useEffect(() => {
    if (streams.length === 0) return;
    App.addStreamListener(
      streams,
      ["text", "numeric", "bitset"],
      handleStreams
    );
  }, [streams]);

  const handleStreams = (
    currentValue:
      | IStreamData<keyof IStreamTypeMap>[]
      | "too much data"
      | undefined
  ) => {
    if (!currentValue) return;
    if (currentValue.length === 0) {
      setCurrentValues({});
      return;
    }

    const reducedValues = (currentValue as IStreamData[]).reduce<any>(
      (prev, currentStream) => {
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
      },
      {}
    );
    setCurrentValues(reducedValues);
  };

  return currentValues;
};
