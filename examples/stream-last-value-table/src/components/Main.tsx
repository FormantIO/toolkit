import { Table } from "./Table";
import { useFormant } from "@formant/ui-sdk";
import { useCurrentStreamsValues } from "hooks/useCurrentStreamsValues";
import { useMemo } from "react";
import { IConfiguration, IReducedConfiguration } from "types";
import {
  handleReduceConfigurationStreams,
  listStreamNames,
  dummyData,
} from "utils/utils";

export const Main = () => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;

  const reduceStreams = (streams: IConfiguration) =>
    Object.entries(streams).reduce((prev, current) => {
      if (current[0] === "fullScreenMode") return prev;
      const streamsType: any = current[0];
      const streams: any[] = current[1];
      const reucedStreams = handleReduceConfigurationStreams(
        streams,
        streamsType
      );
      return { ...prev, ...reucedStreams };
    }, {});

  const areStreamsConfigured = (s: IConfiguration) =>
    Object.entries(s).filter(
      (current) => current[0] !== "fullScreenMode" && current[1].length > 0
    ).length < 1;

  const reducedConfiguration: IReducedConfiguration = useMemo(() => {
    if (!config || areStreamsConfigured(config))
      return reduceStreams(dummyData);

    return reduceStreams(config);
  }, [config]);

  const streams = useCurrentStreamsValues(
    listStreamNames(reducedConfiguration)
  );

  return (
    <Table
      currentConfiguration={reducedConfiguration}
      currentValues={streams}
    />
  );
};
