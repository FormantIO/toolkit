import { Header } from "./Header";
import { Row } from "./Row/index";
import { FC } from "react";
import { ICurrentValues, IReducedConfiguration } from "../types";
import { handleTableCurrentValue, handleStreamStatus } from "../utils/utils";
interface IMainProps {
  currentConfiguration: IReducedConfiguration;
  currentValues: ICurrentValues;
}

export const Table: FC<IMainProps> = ({
  currentConfiguration,
  currentValues,
}) => {
  return (
    <>
      <Header />
      {Object.entries(currentConfiguration)
        .sort()
        .map((s) => {
          const streamName = s[0];
          return (
            <Row
              key={streamName}
              leftValue={streamName}
              rightValue={handleTableCurrentValue(currentValues, streamName)}
              state={handleStreamStatus(
                currentValues,
                streamName,
                currentConfiguration
              )}
            />
          );
        })}
    </>
  );
};
