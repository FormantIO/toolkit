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
  return stream.expectedValue.reduce<any>((prev, current) => {
    prev[`${stream.name}/${current.key}`] = current.value;
    return prev;
  }, {});
};

export const handleReduceConfigurationStreams = (
  streams: (
    | IBitsetConfiguration
    | INumericConfiguration
    | ITextConfiguration
  )[],
  streamType: StreamConfigurationType
) => {
  return streams.reduce<any>((prevactualStream, currentActualStream) => {
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
