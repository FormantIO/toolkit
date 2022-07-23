import { useLatestTelemetry, JsonSchemaForm, Button } from "@formant/ui-sdk";
import { Authentication, KeyValue } from "@formant/data-sdk";
import { useEffect, useState, useMemo } from "react";
import { Row } from "./Row";
import { Header } from "./Header";
import { Configuration } from "./Configuration";

export interface LastKnowValue {
  currentValue: string;
  currentValueTime: string;
  deviceId: string;
  id: string;
  streamName: string;
  streamType: string;
}

export type configuration = {
  streamName: string;
  enable: boolean;
};

const getValue = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    return JSON.parse(await KeyValue.get("lastKnowValuesList"));
  }
};

export const Table = () => {
  const telemetry = useLatestTelemetry();
  const [showConfig, setShowCongif] = useState(false);
  const [currentConfiguration, setCurrentConfiguration] = useState();
  const [lastKnowValueList, setLastKnowValueList] = useState<LastKnowValue[]>(
    []
  );

  useEffect(() => {
    if (!telemetry) return;
    setLastKnowValueList(
      telemetry.filter((_: LastKnowValue) => _.streamType === "text")
    );
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
