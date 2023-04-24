import { Header } from "./Header";
import { Row } from "./Row/index";
import { FC, useEffect, useMemo } from "react";
import {
  ICurrentValues,
  IReducedConfiguration,
  IConfiguration,
} from "../types";
import { handleTableCurrentValue, handleStreamStatus } from "../utils/utils";
import { set, unset } from "lodash";
import { useFormant } from "@formant/ui-sdk";

interface IMainProps {
  currentConfiguration: IReducedConfiguration;
  currentValues: ICurrentValues;
}

export const Table: FC<IMainProps> = ({
  currentConfiguration,
  currentValues,
}) => {
  const context = useFormant();
  const config = context.configuration as IConfiguration;

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

  const numericSetsFilter = useMemo(() => {
    return Object.keys(currentValues).filter((_) =>
      Array.isArray(currentValues[_])
    );
  }, [currentValues]);

  const subsets = useMemo(() => {
    const numericSets = Object.entries(currentValues).filter((_) =>
      Array.isArray(_[1])
    );

    const s = numericSets.reduce((p, c) => {
      p[c[0]] = c[1];
      return p;
    }, {});
    return numericSets;
  }, [currentValues]);

  const widths = useMemo(() => {
    const fullWidthStreams: string[] = [];
    Object.values(config ?? {}).forEach((s) => {
      //Avoids teleop confiruation error
      if (!Array.isArray(s)) return;
      s.forEach((stream: any) => {
        if (stream.fullwidth) {
          fullWidthStreams.push(stream.name);
        }
      });
    });

    return fullWidthStreams;
  }, [config]);

  return (
    <div style={{ display: "flex", flexWrap: "wrap" }}>
      <Header height={config.rowHeight} />
      {subsets.map((stream) => {
        return stream.map((s, idx) => {
          if (idx === 0) {
            return (
              <Row
                key={s}
                leftValue={s}
                rightValue={"Header"}
                fullWidth={true}
                height={config.rowHeight}
                state={"good"}
              />
            );
          }
          return s.map((_) => (
            <Row
              key={Object.keys(_)}
              leftValue={Object.keys(_)}
              rightValue={Object.values(_)}
              fullWidth={true}
              height={config.rowHeight}
              state={"good"}
            />
          ));
        });
      })}
      {Object.entries(configuration)
        .sort()
        .filter((str) => widths.includes(str[0]))
        .map((s) => {
          const streamName = s[0];
          if (numericSetsFilter.includes(streamName)) return <></>;
          return (
            <Row
              key={streamName}
              leftValue={streamName}
              rightValue={handleTableCurrentValue(currentValues, streamName)}
              fullWidth={widths.includes(streamName)}
              height={config.rowHeight}
              state={handleStreamStatus(
                currentValues,
                streamName,
                currentConfiguration
              )}
            />
          );
        })}
      {Object.entries(configuration)
        .sort()
        .filter((str) => !widths.includes(str[0]))
        .map((s, idx) => {
          const streamName = s[0];
          if (numericSetsFilter.includes(streamName)) return <></>;

          return (
            <Row
              idx={idx}
              key={streamName}
              leftValue={streamName}
              height={config.rowHeight}
              rightValue={handleTableCurrentValue(currentValues, streamName)}
              fullWidth={widths.includes(streamName)}
              state={handleStreamStatus(
                currentValues,
                streamName,
                currentConfiguration
              )}
            />
          );
        })}
    </div>
  );
};
