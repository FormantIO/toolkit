import {
  Typography,
  Switch,
  Box,
  Icon,
  useDevice,
  TextField,
} from "@formant/ui-sdk";
import { FC, useEffect } from "react";
import { configuration } from "./types";
import { Footer } from "./Footer";
import { KeyValue, Authentication } from "@formant/data-sdk";
import { useState, useCallback, ChangeEventHandler } from "react";
import "./App.css";

interface IConfigurationProps {
  onBack: () => void;
  streams: any[];
  currentConfiguration: {
    [key: string]: {
      enabled: boolean;
      expectedValue: string;
    };
  };
}

export const Configuration: FC<IConfigurationProps> = ({
  onBack,
  streams,
  currentConfiguration,
}) => {
  const [streamList, setStreamsList] = useState<{
    [key: string]: { enabled: boolean; expectedValue: string };
  }>({});

  useEffect(() => {
    console.log(currentConfiguration);
    if (
      currentConfiguration === undefined ||
      (currentConfiguration as any).length === 0
    ) {
      //If not configuration has been set all the streams are set to true
      console.log("here");
      setStreamsList(
        Object.keys(streams).reduce((_, item) => {
          return { ..._, [item]: { enabled: true, expectedValue: "" } };
        }, {})
      );
    }
    if (Object.keys(currentConfiguration).length > 0) {
      //If new streams is added, it is set to true as first value

      console.log(currentConfiguration);
      let x = Object.keys(streams).reduce((_, item) => {
        return {
          ..._,
          [item]: {
            expectedValue: currentConfiguration[item].expectedValue ?? "",
            enabled: currentConfiguration[item as any].enabled ?? true,
          },
        };
      }, {});
      console.log(x);
      setStreamsList(x);
    }
  }, [currentConfiguration]);

  const handleChange = useCallback(
    async (e: any, index: number) => {
      //toggle stream state
      setStreamsList({
        ...streamList,
        [e.target.value]: {
          ...streamList[e.target.value],
          enabled: !streamList[e.target.value].enabled,
        },
      });
      const changeList = {
        ...streamList,
        [e.target.value]: {
          ...streamList[e.target.value],
          enabled: !streamList[e.target.value].enabled,
        },
      };

      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set("lastKnowValuesList", JSON.stringify(changeList));
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
        await KeyValue.set("lastKnowValuesList", JSON.stringify(changeList));
      }
    },
    [streams, streamList]
  );

  const handleSubmit = useCallback(async () => {
    console.log(streamList);
  }, [streamList]);

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
        borderRadius={25}
        width="100vw"
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
            onClick={onBack}
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
        {Object.keys(streams).map((_, index) => {
          //Prevents undefined error for check property of 'switch' component
          if (Object.keys(streamList).length === 0) return;

          return (
            <Box>
              <Box
                sx={{
                  display: "flex",
                  width: 224,
                  justifyContent: "space-between",
                  alignItems: "center",
                }}
              >
                <TextField
                  onChange={(ev) => handleExpectedValueChange(ev, _)}
                  variant="standard"
                  value={streamList[_].expectedValue}
                  label={_}
                />
                {/* <Box
                  sx={{
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "space-between",
                  }}
                >
                  <Typography
                    sx={{
                      marginRight: 1,
                    }}
                    variant="h6"
                  >
                    Stream name:{" "}
                  </Typography>
                  <input className="treshhold" />
                  <span>{"< x < "}</span>
                  <input className="treshhold" />
                </Box> */}

                <Switch
                  value={_}
                  onChange={(e) => handleChange(e, index)}
                  checked={streamList[_].enabled}
                  size="small"
                />
              </Box>
            </Box>
          );
        })}
      </Box>
      {/* <Footer label={"Save"} onClick={handleSubmit} /> */}
    </Box>
  );
};
