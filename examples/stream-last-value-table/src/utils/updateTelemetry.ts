import { Authentication, Device } from "@formant/data-sdk";
import { Text, Numeric, Bitset } from "../types";

export const updateTelemetry = async (
  device: Device,
  setterFunction: (_: any) => void,
  expirationTime: number
) => {
  while (getShowConfig() === "false") {
    if (await Authentication.waitTilAuthenticated()) {
      const currentTime = Date.now();
      const tel = await device.getLatestTelemetry();
      const latestTel = tel.reduce(
        (_: any, stream: Text | Numeric | Bitset) => {
          if (stream.streamType === "text" || stream.streamType === "numeric") {
            if (
              stream.streamName in _ &&
              _[stream.streamName].currentValueTime >
                new Date(stream.currentValueTime).getTime()
            )
              return _;

            _[stream.streamName] = {
              currentValue:
                currentTime - new Date(stream.currentValueTime).getTime() >
                  expirationTime && expirationTime !== 0
                  ? "-"
                  : stream.currentValue,
              streamType: stream.streamType,
              currentValueTime: new Date(stream.currentValueTime).getTime(),
            };
            return _;
          }
          if (stream.streamType === "bitset") {
            if (stream.streamName === "$.ros.node_online") return _;

            if (
              (stream.currentValue as { keys: string[]; values: boolean[] })
                .keys.length === 1
            ) {
              if (
                stream.streamName in _ &&
                _[stream.streamName].currentValueTime >
                  new Date(stream.currentValueTime).getTime()
              )
                return _;
              _[stream.streamName] = {
                currentValue:
                  currentTime - new Date(stream.currentValueTime).getTime() >
                    expirationTime && expirationTime !== 0
                    ? "-"
                    : (stream as Bitset).currentValue.values[0],
                streamType: stream.streamType,
                currentValueTime: new Date(stream.currentValueTime).getTime(),
              };
              return _;
            }
            let x = (
              stream.currentValue as { keys: string[]; values: boolean[] }
            ).keys.reduce((bitset, bit, idx) => {
              if (
                bit in _ &&
                _[bit].currentValueTime >
                  new Date(stream.currentValueTime).getTime()
              )
                return _;

              // _[bit] =  {
              //   currentValue:
              //     currentTime -
              //       new Date(stream.currentValueTime).getTime() >
              //     MINUTE_IN_MILLISECONDS * 2
              //       ? "-"
              //       : (stream as Bitset).currentValue.values[idx],
              //   streamType: "bitset",
              //   currentValueTime: new Date(
              //     stream.currentValueTime
              //   ).getTime(),
              // }
              return {
                ...bitset,
                [bit]: {
                  currentValue:
                    currentTime - new Date(stream.currentValueTime).getTime() >
                      expirationTime && expirationTime !== 0
                      ? "-"
                      : (stream as Bitset).currentValue.values[idx],
                  streamType: "bitset",
                  currentValueTime: new Date(stream.currentValueTime).getTime(),
                },
              };
            }, {});
            return {
              ..._,
              ...x,
            };
          }
          return _;
        },
        {}
      );
      setterFunction(latestTel);
    }
    await timeout(1000);
  }
};

export const timeout = (ms: number) => {
  return new Promise((resolve) => setTimeout(resolve, ms));
};

const getShowConfig = () => localStorage.getItem("show");
