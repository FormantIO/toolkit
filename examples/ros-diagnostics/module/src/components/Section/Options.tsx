import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";

interface IOptions {
  sections: string[];
}

export const Options: FC<IOptions> = ({ sections }) => {
  return (
    <Box
      sx={{
        backgroundColor: "black",
        border: "1px solid #18D2FF",
        borderRadius: 0.4,
        boxSizing: "border-box",
      }}
    >
      <Box
        sx={{
          boxSizing: "border-box",
          width: "180px",
          height: "40px",
          display: "Flex",
          flexDirection: "column",
          justifyContent: "center",
        }}
      >
        <Typography
          sx={{
            paddingLeft: 1,
            color: "white",
          }}
        >
          Move to
        </Typography>
      </Box>
      {sections.map((_) => (
        <Option label={_} />
      ))}
      <Option label="Delete" />
    </Box>
  );
};

interface IOption {
  label: string;
}

const Option: FC<IOption> = ({ label }) => {
  return (
    <Box
      sx={{
        boxSizing: "border-box",
        width: "180px",
        height: "40px",
        display: "Flex",
        flexDirection: "column",
        justifyContent: "center",
        " :hover": {
          backgroundColor: "#3B4668",
          cursor: "pointer",
        },
      }}
    >
      <Typography
        sx={{
          paddingLeft: 1,
        }}
      >
        {label}
      </Typography>
    </Box>
  );
};
