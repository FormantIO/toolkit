import { Select, TextField } from "@formant/ui-sdk";
import { Authentication } from "@formant/data-sdk";
import { baseUrl } from "./config";
import { FC, useEffect, useMemo, useState } from "react";
import {
  AggregatePeriod,
  AggregateType,
  IAggregateConfiguration,
} from "./types";

const getAvailableStreams = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    try {
      const result = await fetch(`${baseUrl}/streams`, {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      });
      const parseResult = await result.json();
      if (parseResult.message) return null;

      const items = parseResult.items.filter(
        (_: { streamType: string; active: boolean }) =>
          _.active &&
          (_.streamType === "numeric set" || _.streamType === "numeric")
      );

      return items;
    } catch {
      return null;
    }
  }

  return null;
};

interface IProps {
  state: IAggregateConfiguration;
  setNumberAgg: (_: number) => void;
  setType: (_: AggregateType) => void;
  setPeriod: (_: AggregatePeriod) => void;
  setStreamName: (_: string) => void;
  setStreamKey: (_: string) => void;
}

export const Configuration: FC<IProps> = ({
  state,
  setNumberAgg,
  setType,
  setPeriod,
  setStreamName,
  setStreamKey,
}) => {
  const [streams, setStreams] = useState<any>(null);

  useEffect(() => {
    getAvailableStreams().then((_) => setStreams(_));
  }, []);
  const streamList = useMemo(() => {
    if (!streams) return [];

    const list = streams.map((_: { streamName: string }) => ({
      value: _.streamName,
      label: _.streamName,
    }));
    return list;
  }, [streams]);

  const aggregateTypes = [
    { value: "min", label: "min" },
    {
      value: "max",
      label: "max",
    },
    {
      value: "standard deviation",
      label: "standard deviation",
    },
    { value: "average", label: "average" },
    { value: "sum", label: "sum" },
    { value: "count", label: "count" },
  ];

  const aggregateBy = [
    { value: "day", label: "day" },
    { value: "week", label: "week" },
    { value: "month", label: "month" },
  ];

  return (
    <div
      style={{
        minWidth: "100vw",
        minHeight: "100vh",
        display: "flex",
        alignItems: "center",
        flexDirection: "column",
        justifyContent: "center",
      }}
    >
      <div>
        <TextField
          variant="filled"
          type={"number"}
          fullWidth={true}
          label="Numer of agregates"
          value={state.numAggregates}
          onChange={(e) => setNumberAgg(parseInt(e.target.value))}
          //TODO: Handle Negatives
        />
        <Select
          sx={{
            textAlign: "left",
          }}
          value={state.aggregateType}
          onChange={(value) => setType(value as AggregateType)}
          items={aggregateTypes}
          label="Type"
        />
        <Select
          sx={{
            textAlign: "left",
          }}
          value={state.aggregateBy}
          onChange={(value) => setPeriod(value as AggregatePeriod)}
          items={aggregateBy}
          label="Aggregate by"
        />
        <Select
          sx={{
            textAlign: "left",
          }}
          value={state.streamName}
          onChange={(values) => setStreamName(values)}
          items={streamList}
          label="Stream Name"
        />
        <TextField
          variant="filled"
          type={"text"}
          fullWidth={true}
          label="Stream Key (Numeric set)"
          value={state.numericSetKey}
          onChange={(e) => setStreamKey(e.target.value)}
        />
      </div>
    </div>
  );
};
