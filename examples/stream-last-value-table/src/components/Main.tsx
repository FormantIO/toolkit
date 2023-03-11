import { Table } from "./Table";
import { useFormant } from "@formant/ui-sdk";
import { useCurrentStreamsValues } from "hooks/useCurrentStreamsValues";
import { useEffect, useMemo, useState } from "react";
import { IConfiguration, IReducedConfiguration } from "types";
import { listStreamNames, dummyData, reduceStreams } from "utils/utils";
import { LoadingIndicator } from "@formant/ui-sdk";

export const Main = () => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const [loading, setLoading] = useState(true);

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

  useEffect(() => {
    setTimeout(() => {
      setLoading(false);
    }, 2000);
  }, []);

  return (
    <>
      {loading ? (
        <LoadingIndicator />
      ) : (
        <Table
          currentConfiguration={reducedConfiguration}
          currentValues={streams}
        />
      )}
    </>
  );
};
