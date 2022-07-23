import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";

interface IRowProps {
  leftValue: string;
  rightValue: string;
}

export const Row: FC<IRowProps> = (props: any) => {
  const { leftValue, rightValue } = props;
  return (
    <Box
      sx={{
        width: "100%",
        heigh: "auto",
        borderBottom: "1px solid black",
        display: "flex",
      }}
    >
      <Box
        sx={{
          width: "30%",
          padding: 1,
        }}
      >
        <Typography variant="body2" sx={{ color: "white", fontSize: 14 }}>
          {leftValue}
        </Typography>
      </Box>
      <Box
        sx={{
          padding: 1,
        }}
      >
        <Typography variant="body2" sx={{ color: "white", fontSize: 14 }}>
          {rightValue}
        </Typography>
      </Box>
    </Box>
  );
};
