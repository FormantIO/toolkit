import { Header } from "./Header";
import { Row } from "./Row/index";
import { FC, useEffect, useMemo } from "react";
import { getStreamStatus } from "../utils/getStreamsStatus";
import { useCurrentValues } from "@hooks";
import { Configuration } from "../types";
interface IMainProps {
  currentConfiguration: Configuration;
}

//Handle when no configuration,
//Which should not happen

export const Table: FC<IMainProps> = ({ currentConfiguration }) => {
  if (
    currentConfiguration === undefined ||
    currentConfiguration.streams.length === 0
  )
    return <div>Add streams to start</div>;
  const currentValues = useCurrentValues(
    currentConfiguration.streams.map((_) => _.name)
  );
  const configurationLength = useMemo(() => {
    return currentConfiguration.streams.length;
  }, [currentConfiguration]);

  const rowHeight = useMemo(() => {
    if (window.innerWidth < 301 && configurationLength > 5) {
      const TELEOP_MODULE_HEIGHT = window.innerHeight;
      let rows = configurationLength;

      rows =
        rows + currentConfiguration.streams.filter((_) => _.fullWidth).length;

      const HEADER = 1;
      rows = rows % 2 === 0 ? rows / 2 : Math.floor((rows + 1) / 2) + HEADER;

      return (TELEOP_MODULE_HEIGHT - rows) % rows === 0
        ? (TELEOP_MODULE_HEIGHT - rows) / rows
        : Math.floor((TELEOP_MODULE_HEIGHT - rows) / rows) + 1;
    }
    return null;
  }, [currentValues, currentConfiguration]);

  return (
    <>
      <Header height={rowHeight} />
      <div style={{ display: "flex", flexWrap: "wrap" }}>
        {currentConfiguration.streams.sort().map((_) => {
          if (_.streamType === "bitset") {
            return _.expectedValue.keys.map((bit: string, idx: string) => {
              return (
                <Row
                  key={bit}
                  fullWidth={_.fullWidth}
                  height={rowHeight}
                  leftValue={bit}
                  rightValue={(currentValues[bit] as string)?.toString() ?? "-"}
                  state={getStreamStatus(currentConfiguration, currentValues, {
                    name: bit,
                    streamType: _.streamType,
                    expectedValue: _.expectedValue.values[idx],
                    fullWidth: false,
                  })}
                />
              );
            });
          }
          return (
            <Row
              key={_.name}
              fullWidth={_.fullWidth}
              height={rowHeight}
              leftValue={_.name}
              rightValue={(currentValues[_.name] as string) ?? "-"}
              state={getStreamStatus(currentConfiguration, currentValues, _)}
            />
          );
        })}
      </div>
    </>
  );
};
