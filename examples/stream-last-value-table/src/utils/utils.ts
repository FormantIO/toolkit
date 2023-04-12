import { INumericSetEntry } from "@formant/data-sdk";
import {
  StreamConfigurationType,
  IBitsetConfiguration,
  INumericConfiguration,
  ITextConfiguration,
  IReducedConfiguration,
  ICurrentValues,
  Status,
  InumericExpectedValue,
  IConfiguration,
  INumericSetConfiguration,
} from "types";

export const listStreamNames = (streams: IReducedConfiguration) => [
  ...new Set(
    Object.keys(streams).map((_) => {
      if (_.includes("/")) {
        const idx = _.indexOf("/");
        return _.slice(0, idx);
      }
      return _;
    })
  ),
];

const handleBitsetStreams = (stream: IBitsetConfiguration) => {
  if ("expectedValue" in stream) {
    return stream.expectedValue.reduce<any>((prev, current) => {
      prev[`${stream.name}/${current.key}`] = current.value;
      return prev;
    }, {});
  } else {
    return {
      [`${(stream as IBitsetConfiguration).name}/undefined`]: undefined,
    };
  }
};

export const handleReduceConfigurationStreams = (
  streams: (
    | IBitsetConfiguration
    | INumericConfiguration
    | ITextConfiguration
    | INumericSetConfiguration
  )[],
  streamType: StreamConfigurationType
) => {
  const x = streams.reduce<any>((prevactualStream, currentActualStream) => {

    if(streamType === "numericSetStreams"){

    }

    if (streamType === "textStreams") {
      prevactualStream[currentActualStream.name] = (
        currentActualStream as ITextConfiguration
      ).expectedValue;
      return prevactualStream;
    }
    if (streamType === "numericStreams") {
      prevactualStream[currentActualStream.name] = {
        greaterThan: (currentActualStream as INumericConfiguration).greaterThan,
        lesserThan: (currentActualStream as INumericConfiguration).lesserThan,
      };
      return prevactualStream;
    }
    const bits = handleBitsetStreams(
      currentActualStream as IBitsetConfiguration
    );
    return { ...prevactualStream, ...bits };
  }, {});

  return x;
};

export const handleStreamStatus = (
  currentValues: ICurrentValues,
  streamName: string,
  configuration: IReducedConfiguration
): Status => {
  if (Object.keys(currentValues).length === 0) return "offline";
  if (!Object.keys(currentValues).includes(streamName)) return "offline";
  if (configuration[streamName] === undefined) return "good";
  if (configuration[streamName] instanceof String) {
    return configuration[streamName] === currentValues[streamName]
      ? "good"
      : "warning";
  }
  if (configuration[streamName] instanceof Boolean) {
    return configuration[streamName] === currentValues[streamName]
      ? "good"
      : "warning";
  }
  if (
    (configuration[streamName] as InumericExpectedValue).greaterThan !==
      undefined &&
    (configuration[streamName] as InumericExpectedValue).greaterThan !==
      undefined
  ) {
    return currentValues[streamName] >
      (configuration[streamName] as InumericExpectedValue).greaterThan &&
      currentValues[streamName] <
        (configuration[streamName] as InumericExpectedValue).greaterThan
      ? "good"
      : "warning";
  }
  if (
    (configuration[streamName] as InumericExpectedValue).greaterThan !==
    undefined
  ) {
    return currentValues[streamName] >
      (configuration[streamName] as InumericExpectedValue).greaterThan
      ? "good"
      : "warning";
  }
  return currentValues[streamName] <
    (configuration[streamName] as InumericExpectedValue).greaterThan
    ? "good"
    : "warning";
};

export const handleTableCurrentValue = (
  currentValues: ICurrentValues,
  streamName: string
) => {
  if (Object.keys(currentValues).length === 0) return "-";
  if (streamName in currentValues) return currentValues[streamName].toString();
  return "-";
};

const sampleTextStreams: ITextConfiguration[] = [
  { name: "Set a stream", expectedValue: "bot" },
];

const sampleNumericStream = [
  {
    name: "To track your data",
    greaterThan: 0,
    lesserThan: 500,
  },
];

const sampleBitsetStream = [
  {
    name: "stream",
    expectedValue: [{ key: "bit.one", value: true }],
  },
];

export const dummyData: IConfiguration = {
  fullScreenMode: true,
  textStreams: sampleTextStreams,
  numericStreams: sampleNumericStream,
  bitsetStreams: sampleBitsetStream,
};

export const reduceStreams = (streams: IConfiguration) => {
  const x = Object.entries(streams).reduce((prev, current) => {
    if (["fullScreenMode", "rowHeight", "fontSize"].includes(current[0]))
      return prev;
    const streamsType: any = current[0];
    const streams: any[] = current[1];
    console.log(streams)
    const reucedStreams = handleReduceConfigurationStreams(
      streams,
      streamsType
    );
    return { ...prev, ...reucedStreams };
  }, {});

  return { ...x };
};
