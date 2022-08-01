import { useLatestTelemetry, JsonSchemaForm, Button } from "@formant/ui-sdk";
import { Authentication, KeyValue } from "@formant/data-sdk";
import { useEffect, useState, useMemo } from "react";
import { Row } from "./Row";
import { Header } from "./Header";
import { Configuration } from "./Configuration";
import { Bitset, Text, Numeric } from "./types";

const getValue = async () => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      return JSON.parse(await KeyValue.get("lastKnowValuesList"));
    }
    return [];
  } catch (e) {
    return [];
  }
};

export const Table = () => {
  const telemetry = useLatestTelemetry();
  const [showConfig, setShowCongif] = useState(false);
  const [bitsetStreams, setBitsetStreams] = useState<Bitset[]>([]);
  const [currentConfiguration, setCurrentConfiguration] = useState<any>();
  const [lastKnowValueList, setLastKnowValueList] = useState<
    {
      [key: string]: {
        expectedValue: string;
        enabled: boolean;
      };
    }[]
  >([]);

  useEffect(() => {
    getValue().then((_) => {
      console.log(_);
      console.log(lastKnowValueList);
      setCurrentConfiguration(_);
    });
  }, [showConfig]);

  useEffect(() => {
    if (!telemetry) return;

    const textAndNumericStreams = telemetry.reduce(
      (_: any, stream: Text | Numeric | Bitset) => {
        if (stream.streamType === "text" || stream.streamType === "numeric") {
          return { ..._, [stream.streamName]: stream.currentValue };
        }
        if (stream.streamType === "bitset") {
          if (stream.streamName === "$.ros.node_online") return _;

          let x = (
            stream.currentValue as { keys: string[]; values: boolean[] }
          ).keys.reduce((bitset, bit, idx) => {
            return {
              ...bitset,
              [bit]: (stream as Bitset).currentValue.values[idx],
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

    setLastKnowValueList(textAndNumericStreams);
  }, [telemetry, showConfig]);

  return showConfig ? (
    <Configuration
      streams={lastKnowValueList}
      onBack={() => setShowCongif(false)}
      currentConfiguration={currentConfiguration}
    />
  ) : (
    <>
      <Header setShow={() => setShowCongif(true)} />
      {Object.keys(lastKnowValueList).map((_) => {
        if (
          currentConfiguration !== undefined &&
          currentConfiguration[_].enabled === false
        ) {
          return;
        }

        return (
          <Row
            key={_}
            leftValue={_}
            rightValue={lastKnowValueList[_ as any].toString()}
            state={
              lastKnowValueList[_ as any].toString() ===
              currentConfiguration[_].expectedValue
                ? "good"
                : "warning"
            }
          />
        );
      })}
    </>
  );
};
