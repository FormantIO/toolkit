import { Table } from "./Table";
import { useFormant } from "@formant/ui-sdk";
import { useCurrentStreamsValues } from "hooks/useCurrentStreamsValues";
import { useEffect, useMemo, useState } from "react";
import { IConfiguration, IReducedConfiguration } from "types";
import { listStreamNames, dummyData, reduceStreams } from "utils/utils";
import { useScrubberTime } from "@formant/ui-sdk";
export const Main = () => {
  const time = useScrubberTime();
  const context = useFormant();
  const config = context.configuration as IConfiguration;

  const areStreamsConfigured = (s: IConfiguration) =>
    Object.entries(s).filter(
      (current) => current[0] !== "fullScreenMode" && current[1].length > 0
    ).length < 1;

  const reducedConfiguration: IReducedConfiguration = useMemo(() => {
    if (!config || areStreamsConfigured(config)) return {};

    return reduceStreams(config);
  }, [config]);

  const streams = useCurrentStreamsValues(
    listStreamNames(reducedConfiguration),
    time as any
  );

  return (
    <Table
      currentConfiguration={reducedConfiguration}
      currentValues={streams}
    />
  );
};
