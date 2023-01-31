import {
  Typography,
  Switch,
  Box,
  Icon,
  useDevice,
  TextField,
} from "@formant/ui-sdk";
import { FC, useEffect } from "react";
import { Iconfiguration, lastKnowValue } from "../types";
import { Footer } from "../Footer";
import { KeyValue, Authentication } from "@formant/data-sdk";
import { useState, useCallback } from "react";
import { NumericConfiguration } from "./NumericConfiguration";
import { ExpirationConfiguation } from "./ExpirationConfiguration";
import { useRecoilValue, useRecoilState } from "recoil";
import {
  moduleNameState,
  showConfigurationState,
} from "../atoms/configuration";
import "../App.css";
import { ThreeWaySwitch } from "./ThreeWaySwitch/index";
interface IConfigurationProps {
  streams: lastKnowValue[];
  currentConfiguration: Iconfiguration;
}

export const Configuration: FC<IConfigurationProps> = ({
  streams,
  currentConfiguration,
}) => {
  const moduleName = useRecoilValue(moduleNameState);
  const [teleopMode, setTeleopMode] = useState(false);
  const [showConfiguration, setShowConfiguration] = useRecoilState(
    showConfigurationState
  );
  const [streamList, setStreamsList] = useState<{
    [key: string]: { enabled: boolean; expectedValue: any; fullWidth: boolean };
  }>({});

  useEffect(() => {
    if (
      currentConfiguration === undefined ||
      (currentConfiguration as any).length === 0
    ) {
      //If not configuration has been set all the streams are set to true
      setStreamsList(
        Object.keys(streams).reduce((_, item) => {
          return {
            ..._,
            [item]: { enabled: true, expectedValue: "", fullWidth: false },
          };
        }, {})
      );
    }
    if (Object.keys(currentConfiguration).length > 0) {
      //If new streams is added, it is set to true as first value

      let x = Object.keys(streams).reduce((_, item) => {
        return {
          ..._,
          [item]: {
            expectedValue:
              currentConfiguration[item] !== undefined
                ? currentConfiguration[item].expectedValue
                : "",
            enabled:
              currentConfiguration[item as any] !== undefined
                ? currentConfiguration[item as any].enabled
                : true,
            fullWidth:
              currentConfiguration[item as any] !== undefined
                ? currentConfiguration[item as any].fullWidth
                : false,
          },
        };
      }, {});
      setStreamsList(x);
    }
  }, [currentConfiguration]);

  const handleFullWidthChange = useCallback(
    async (_: string, size: string) => {
      //toggle stream state
      setStreamsList({
        ...streamList,
        [_]: {
          ...streamList[_],
          enabled: true,
          fullWidth: size === "wide" ? true : false,
        },
      });
      const changeList = {
        ...streamList,
        [_]: {
          ...streamList[_],
          enabled: true,
          fullWidth: size === "wide" ? true : false,
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set(moduleName, JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleOnHide = useCallback(
    async (_: string) => {
      //toggle stream state
      setStreamsList({
        ...streamList,
        [_]: {
          ...streamList[_],
          enabled: false,
        },
      });
      const changeList = {
        ...streamList,
        [_]: {
          ...streamList[_],
          enabled: false,
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set(moduleName, JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleExpectedValueChange = useCallback<any>(
    async (e: any, stream: string) => {
      setStreamsList({
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: e.target.value,
        },
      });
      const changeList = {
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: e.target.value,
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set(moduleName, JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleGreaterThanChange = useCallback<any>(
    async (e: any, stream: string) => {
      setStreamsList({
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: {
            ...streamList[stream].expectedValue,
            greaterThan: e.target.value,
          },
        },
      });
      const changeList = {
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: {
            ...streamList[stream].expectedValue,
            greaterThan: e.target.value,
          },
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set(moduleName, JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleLesserThanChange = useCallback<any>(
    async (e: any, stream: string) => {
      setStreamsList({
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: {
            ...streamList[stream].expectedValue,
            lesserThan: e.target.value,
          },
        },
      });
      const changeList = {
        ...streamList,
        [stream]: {
          ...streamList[stream],
          expectedValue: {
            ...streamList[stream].expectedValue,
            lesserThan: e.target.value,
          },
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set(moduleName, JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleSubmit = useCallback(async () => {}, [streamList]);

  return (
    <Box
      sx={{
        width: "100vw",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        paddingTop: 5,
      }}
    >
      <Box
        position={"fixed"}
        top={0}
        left={0}
        height={"40px"}
        display={"flex"}
        alignItems="center"
        justifyContent="space-between"
        paddingLeft={1}
        paddingRight={1}
        width="100vw"
        
        sx={{ backgroundColor: " #1c1e2c", zIndex: 10 }}
      >
        <Box display={"flex"} alignItems="center">
          <Box
            height={"30px"}
            width="30px"
            borderRadius={25}
            display={"flex"}
            alignItems="center"
            justifyContent={"center"}
            marginRight={1.5}
            onClick={() => {
              localStorage.setItem("show", "false");
              setShowConfiguration(false);
            }}
            sx={{
              ":hover": {
                backgroundColor: "#3b4668",
                cursor: "pointer",
              },
            }}
          >
            <Icon name="arrow-left" />
          </Box>
        </Box>
      </Box>
      <Box display="flex" flexDirection="column">
        <ExpirationConfiguation />
        {Object.keys(streams)
          .sort()
          .map((_, index) => {
            //Prevents undefined error for check property of 'switch' component
            if (Object.keys(streamList).length === 0) return;

            return (
              <Box key={_}>
                <Box
                  sx={{
                    display: "flex",
                    width: 224,
                    justifyContent: "space-between",
                    alignItems: "center",
                    marginBottom: 1,
                  }}
                >
                  {streams[_ as any].streamType !== "numeric" ? (
                    <TextField
                      onChange={(ev) => handleExpectedValueChange(ev, _)}
                      variant="filled"
                      value={streamList[_].expectedValue}
                      label={_}
                      sx={{
                        width: 200,
                      }}
                    />
                  ) : (
                    <NumericConfiguration
                      streamName={_}
                      lesserThan={streamList[_].expectedValue.lesserThan}
                      greaterThan={streamList[_].expectedValue.greaterThan}
                      handleGreaterThanChange={handleGreaterThanChange}
                      handleLesserThanChange={handleLesserThanChange}
                    />
                  )}
                  <ThreeWaySwitch
                    handleOnHide={() => handleOnHide(_)}
                    handleOnWide={() => handleFullWidthChange(_, "wide")}
                    handleOnNarrow={() => handleFullWidthChange(_, "narrow")}
                    active={
                      streamList[_].enabled
                        ? streamList[_].fullWidth
                          ? "wide"
                          : "narrow"
                        : "hidden"
                    }
                  />
                </Box>
              </Box>
            );
          })}
      </Box>
      {/* <Footer label={"Save"} onClick={() => console.log(streamList)} /> */}
    </Box>
  );
};
