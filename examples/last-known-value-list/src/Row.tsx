import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";

interface IRowProps {
  leftValue: string;
  rightValue: string;
  state: "warning" | "good";
}

export const Row: FC<IRowProps> = (props: any) => {
  const { leftValue, rightValue, state } = props;
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
          width: "40%",
          padding: 1,
        }}
      >
        <Typography
          variant="body2"
          sx={{ color: "white", fontSize: 11, wordBreak: "break-all" }}
        >
          {leftValue}
        </Typography>
      </Box>
      <Box
        sx={[
          {
            padding: 1,
            borderLeft: "1px solid black",
            position: "relative",
          },
          {
            "&:before": {
              content: "''",
              height: 25,
              backgroundColor: state === "good" ? "#58b7de" : "#f89973",
              width: "5px",
              position: "absolute",
              left: 0,
              borderTopRightRadius: 25,
              borderBottomRightRadius: 25,
              marginTop: 0.1,
            },
          },
        ]}
      >
        <Typography
          variant="body2"
          sx={{ color: "white", fontSize: 11, paddingLeft: 1 }}
        >
          {rightValue}
        </Typography>
      </Box>
    </Box>
  );
};
