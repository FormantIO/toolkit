import { Select, TextField } from "@formant/ui-sdk";
import { Authentication } from "@formant/data-sdk";
import { baseUrl } from "./config";
import { FC, useEffect, useMemo, useState } from "react";


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
      //1361

      const items = parseResult.items.filter(
        (_) =>
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
  numberAgg: any;
  setNumberAgg: any;
  type: any;
  setType: any;
  by: any;
  setBy: any;
  streamName: any;
  setStreamName: any;
  streamKey: any;
  setStreamKey: any;
}

export const Configuration: FC<IProps> = ({
  numberAgg,
  setNumberAgg,
  type,
  setType,
  by,
  setBy,
  streamName,
  setStreamName,
  streamKey,
  setStreamKey,
}) => {
  const [streams, setStreams] = useState<any>(null);

  useEffect(() => {
    getAvailableStreams().then((_) => setStreams(_));
  }, []);
  const streamList = useMemo(() => {
    if (!streams) return [];

    const list = streams.map((_) => ({
      value: _.streamName,
      label: _.streamName,
    }));
    console.log(list);
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
          value={numberAgg}
          onChange={(e) => setNumberAgg(e.target.value)}
        />
        <Select
          value={type}
          onChange={(value) => setType(value)}
          items={aggregateTypes}
          label="Type"
        />
        <Select
          value={by}
          onChange={(value) => setBy(value)}
          items={aggregateBy}
          label="Aggregate by"
        />
        <Select
          value={streamName}
          onChange={(values) => setStreamName(values)}
          items={streamList}
          label="Stream Name"
        />
         {/* <Select
          value={streamName}
          onChange={(values) => setStreamName(values)}
          items={streamList}
          label="Key"
        /> */}
      </div>
    </div>
  );
};
