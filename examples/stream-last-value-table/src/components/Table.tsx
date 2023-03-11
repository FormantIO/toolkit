import { Header } from "./Header";
import { Row } from "./Row/index";
import { FC, useMemo } from "react";
import { ICurrentValues, IReducedConfiguration } from "../types";
import { handleTableCurrentValue, handleStreamStatus } from "../utils/utils";
import { set, unset } from "lodash";
interface IMainProps {
  currentConfiguration: IReducedConfiguration;
  currentValues: ICurrentValues;
}

export const Table: FC<IMainProps> = ({
  currentConfiguration,
  currentValues,
}) => {
  const configuration = useMemo(() => {
    //handles unset bitsets
    const undefinedBitsets = Object.keys(currentConfiguration).filter((_) =>
      _.includes("undefined")
    );
    if (undefinedBitsets.length > 0) {
      const bits = undefinedBitsets.map((_) => {
        const idx = _.indexOf("/");
        return _.slice(0, idx);
      });

      const newkeys: string[] = [];

      Object.keys(currentValues).forEach((_) => {
        bits.forEach((bit) => {
          if (_.includes(bit)) {
            newkeys.push(_);
          }
        });
      });

      undefinedBitsets.forEach((_) => unset(currentConfiguration, _));
      newkeys.forEach((_) => set(currentConfiguration, [_], undefined));
    }
    return currentConfiguration;
  }, [currentValues]);

  return (
    <>
      <Header />
      {Object.entries(configuration)

        .sort()
        .map((s) => {
          const streamName = s[0];
          return (
            <Row
              key={streamName}
              leftValue={streamName}
              rightValue={handleTableCurrentValue(currentValues, streamName)}
              fullWidth={true}
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
