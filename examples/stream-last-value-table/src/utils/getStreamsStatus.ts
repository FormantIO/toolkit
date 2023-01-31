import { Iconfiguration, ICurrentValues } from "../types";

export const getStreamStatus = (
  currentConfiguration: { streams: Iconfiguration[] },
  currentValues: ICurrentValues,
  _: Iconfiguration
) => {
  const { streams } = currentConfiguration;

  if (streams === undefined || streams.length === 0) return "offline";

  if (!Object.keys(currentValues).includes(_.name)) return "offline";

  switch (_.streamType) {
    case "text":
      return _.expectedValue === currentValues[_.name] ? "good" : "warning";

    case "bitset":
      return _.expectedValue === currentValues[_.name] ? "good" : "warning";

    case "numeric":
      if (
        _.expectedValue.greaterThan === null &&
        _.expectedValue.lesserThan === null
      )
        return "good";
      if (
        _.expectedValue.greaterThan === null &&
        currentValues[_.name] < _.expectedValue.lesserThan
      )
        return "good";
      if (
        _.expectedValue.greaterThan < currentValues[_.name] &&
        _.expectedValue.lesserThan === null
      )
        return "good";
      if (
        _.expectedValue.greaterThan < currentValues[_.name] &&
        currentValues[_.name] < _.expectedValue.lesserThan
      ) {
        return "good";
      }

      return "warning";
  }

  return "warning";
};
