import { Table } from "./Table";
import { useFormant } from "@formant/ui-sdk";
import { useCurrentStreamsValues } from "hooks/useCurrentStreamsValues";
import { useMemo } from "react";
import { IConfiguration, IReducedConfiguration } from "types";
import { handleReduceConfigurationStreams, listStreamNames } from "utils/utils";

export const Main = () => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;

  const reducedConfiguration: IReducedConfiguration = useMemo(() => {
    if (!config) return {};

    return Object.entries(config).reduce((prev, current) => {
      const streamsType: any = current[0];
      const streams: any[] = current[1];
      const reucedStreams = handleReduceConfigurationStreams(
        streams,
        streamsType
      );
      return { ...prev, ...reucedStreams };
    }, {});
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
