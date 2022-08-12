import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";

interface IOption {
  label: string;
  path: string;
  handleMoveToSection: (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    _: string
  ) => void;
}

export const Option: FC<IOption> = ({ label, handleMoveToSection, path }) => {
  return (
    <Box
      onClick={(e) => handleMoveToSection(e, path)}
      sx={{
        boxSizing: "border-box",
        height: "40px",
        display: "Flex",
        flexDirection: "column",
        justifyContent: "center",
        borderTop: label === "Delete" ? "1px solid #677194" : "",
        " :hover": {
          backgroundColor: "#3B4668",
          cursor: "pointer",
        },
      }}
    >
      <Typography
        sx={{
          paddingLeft: 2,
          paddingRight: 2,
          color: "white",
        }}
      >
        {label}
      </Typography>
    </Box>
  );
};
