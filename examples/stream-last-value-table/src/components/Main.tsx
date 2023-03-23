import { Table } from "./Table";
import { useFormant } from "@formant/ui-sdk";
import { useCurrentStreamsValues } from "hooks/useCurrentStreamsValues";
import { useEffect, useMemo, useState } from "react";
import { IConfiguration, IReducedConfiguration } from "types";
import { listStreamNames, dummyData, reduceStreams } from "utils/utils";

export const Main = () => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;

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
