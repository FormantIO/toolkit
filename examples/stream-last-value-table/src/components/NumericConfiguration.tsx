import { Box, Typography, TextField } from "@formant/ui-sdk";
import React, { FC } from "react";

interface INumericConfigurationProps {
  handleGreaterThanChange: (
    ev: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>,
    _: string
  ) => void;
  handleLesserThanChange: (
    ev: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>,
    _: string
  ) => void;
  streamName: string;
  greaterThan: number | string;
  lesserThan: number | string;
}

export const NumericConfiguration: FC<INumericConfigurationProps> = ({
  handleGreaterThanChange,
  handleLesserThanChange,
  streamName,
  greaterThan,
  lesserThan,
}) => {
  return (
    <Box
      sx={{
        display: "flex",
        alignItems: "left",
        justifyContent: "space-between",
        // height: 48,
        width: "100%",
        flexDirection: "column",
        marginTop: 2,
      }}
    >
      <Typography
        sx={{
          marginRight: 1,
          fontSize: 14,
        }}
      >
        {`${streamName}: `}
      </Typography>
      <Box
        sx={{
          width: "100%",
          display: "flex",
          alighitems: "center",
          justifyContent: "center",
        }}
      >
        <TextField
          onChange={(ev) => handleGreaterThanChange(ev, streamName)}
          variant="filled"
          value={greaterThan}
          label={"min"}
        />
        <TextField
          onChange={(ev) => handleLesserThanChange(ev, streamName)}
          variant="filled"
          value={lesserThan}
          label={"max"}
        />
      </Box>
    </Box>
  );
};
