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
      setStreamsList(
        streams.reduce((_, item) => {
          return { ..._, [item.streamName]: true };
        }, {})
      );
    } else {
      setStreamsList(currentConfiguration);
    }
  }, [currentConfiguration]);

  const handleChange = useCallback(
    (e: any, index: number) => {
      //toggle stream state
      setStreamsList({
        ...stremasList,
        [e.target.value]: !stremasList[e.target.value],
      });
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
        paddingTop: 10,
      }}
    >
      <Box
        position={"fixed"}
        top={0}
        left={0}
        height={"70px"}
        display={"flex"}
        alignItems="center"
        justifyContent="space-between"
        paddingLeft={2}
        paddingRight={2}
        borderRadius={25}
        width="100vw"
      >
        <Box display={"flex"} alignItems="center">
          <Box
            height={"40px"}
            width="40px"
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
              width: 200,
              justifyContent: "space-between",
            }}
          >
            <Typography sx={{ fontSize: 14 }}>{`${_.streamName}: `}</Typography>{" "}
            <Switch
              value={_.streamName}
              onChange={(e) => handleChange(e, index)}
              checked={(stremasList[_.streamName] as boolean) ?? false}
              size="small"
            />
          </Box>
        ))}
      </Box>
      <Footer label={"Save"} onClick={handleSubmit} />
    </Box>
  );
};
