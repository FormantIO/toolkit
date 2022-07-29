import { useLatestTelemetry, JsonSchemaForm, Button } from "@formant/ui-sdk";
import { Authentication, KeyValue } from "@formant/data-sdk";
import { useEffect, useState, useMemo } from "react";
import { Row } from "./Row";
import { Header } from "./Header";
import { Configuration } from "./Configuration";
import { Bitset, Text, Numeric } from "./types";

const getValue = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    return JSON.parse(await KeyValue.get("lastKnowValuesList"));
  }
};

export const Table = () => {
  const telemetry = useLatestTelemetry();
  const [showConfig, setShowCongif] = useState(false);
  const [currentConfiguration, setCurrentConfiguration] = useState();
  const [lastKnowValueList, setLastKnowValueList] = useState<
    (Text | Numeric)[]
  >([]);

  useEffect(() => {
    if (!telemetry) return;
    const textAndNumericStreams = telemetry.filter(
      (_: Text | Numeric) =>
        _.streamType === "text" || _.streamType === "numeric"
    );
    const bitsetStreams = telemetry.filter((_: Bitset) => {
      if (_.streamType === "bitset") {
        return _.currentValue.keys.reduce((prev, key, idx) => {
          return {
            ...prev,
            [key]: _.currentValue.values[idx],
          };
        }, {});
      }
    });
    console.log(textAndNumericStreams, bitsetStreams);
    setLastKnowValueList(textAndNumericStreams);
    getValue().then((_) => setCurrentConfiguration(_));
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
      {lastKnowValueList.map((_) => {
        if (
          currentConfiguration !== undefined &&
          currentConfiguration[_.streamName] === false
        ) {
          return;
        }
        return (
          <Row
            key={_.id}
            leftValue={_.streamName}
            rightValue={_.currentValue}
          />
        );
      })}
    </>
  );
};
