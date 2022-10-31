import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";

interface INullComponentProps {
  name: string;
  value: string;
}

export const NullComponent: FC<INullComponentProps> = ({ name, value }) => {
  return (
    <Box
      sx={{
        display: "flex",
        borderBottom: "1px solid black",
      }}
    >
      <Typography
        sx={{
          borderRight: "1px solid black",
          padding: 1,
          minWidth: 200,
          color: "white",
        }}
        variant="body1"
      >
        {name}
      </Typography>
      <Typography color={"white"} padding={1} variant="body1">
        {value}
      </Typography>
    </Box>
  );
};
