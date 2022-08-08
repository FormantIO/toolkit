import { Box, TextField, Icon } from "@formant/ui-sdk";
import { FC } from "react";

interface ITopicConfigurationProps {
  name: string;
  type: string;
  hz: number;
}

export const TopicConfiguration: FC<ITopicConfigurationProps> = ({
  name,
  type,
  hz,
}) => {
  return (
    <Box
      sx={{
        display: "flex",
        minWidth: 700,
        justifyContent: "space-between",
        alignItems: "center",
        marginBottom: 1.6,
        height: 57,
      }}
      display={"flex"}
    >
      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Topic"}
        variant="filled"
        value={name}
      />
      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Type"}
        variant="filled"
        value={type}
      />
      <TextField
        sx={{ marginRight: 1, width: 100 }}
        type="text"
        label={"Hz"}
        variant="filled"
        value={Math.floor(hz)}
      />
      <Box
        sx={{
          height: 35,
          width: 40,
          borderRadius: 25,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          ":hover": {
            backgroundColor: "#3b4668",
            cursor: "pointer",
          },
        }}
      >
        <Icon name="more" />
      </Box>
    </Box>
  );
};
