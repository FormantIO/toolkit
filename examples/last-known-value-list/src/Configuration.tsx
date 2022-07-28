import { Typography, Switch, Box, Icon, useDevice } from "@formant/ui-sdk";
import { FC, useEffect } from "react";
import { LastKnowValue, configuration } from "./Table";
import { Footer } from "./Footer";
import { KeyValue, Authentication } from "@formant/data-sdk";
import { useState, useCallback } from "react";

interface IConfigurationProps {
  onBack: () => void;
  streams: LastKnowValue[];
  currentConfiguration?: { [key: string]: Boolean };
}

export const Configuration: FC<IConfigurationProps> = ({
  onBack,
  streams,
  currentConfiguration,
}) => {
  const [stremasList, setStreamsList] = useState<{ [key: string]: Boolean }>(
    {}
  );

  useEffect(() => {
    if (currentConfiguration === undefined) {
      //If not configuration has been set all the streams are set to true
      setStreamsList(
        streams.reduce((_, item) => {
          return { ..._, [item.streamName]: true };
        }, {})
      );
    } else {
      //If new streams is added, it is set to true as first value
      setStreamsList(
        streams.reduce((_, item) => {
          return {
            ..._,
            [item.streamName]: Object.keys(currentConfiguration).includes(
              item.streamName
            )
              ? currentConfiguration[item.streamName]
              : true,
          };
        }, {})
      );
    }
  }, [currentConfiguration]);

  const handleChange = useCallback(
    async (e: any, index: number) => {
      //toggle stream state
      setStreamsList({
        ...stremasList,
        [e.target.value]: !stremasList[e.target.value],
      });
      const changeList = {
        ...stremasList,
        [e.target.value]: !stremasList[e.target.value],
      };
      if (await Authentication.waitTilAuthenticated()) {
        await KeyValue.set("lastKnowValuesList", JSON.stringify(changeList));
      }
    },
    [streams, stremasList]
  );

  const handleSubmit = useCallback(async () => {
    if (await Authentication.waitTilAuthenticated()) {
      await KeyValue.set("lastKnowValuesList", JSON.stringify(stremasList));
      onBack();
    }
  }, [stremasList]);

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
        paddingRight={2}
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
        {streams.map((_, index) => (
          <Box
            sx={{
              display: "flex",
              width: 210,
              justifyContent: "space-between",
            }}
          >
            <Typography sx={{ fontSize: 11 }}>{`${_.streamName}: `}</Typography>{" "}
            <Switch
              value={_.streamName}
              onChange={(e) => handleChange(e, index)}
              checked={(stremasList[_.streamName] as boolean) ?? false}
              size="small"
            />
          </Box>
        ))}
      </Box>
      {/* <Footer label={"Save"} onClick={handleSubmit} /> */}
    </Box>
  );
};
