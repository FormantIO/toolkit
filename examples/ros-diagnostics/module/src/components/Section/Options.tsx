import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import { Option } from "./Option";

interface IOptions {
  sections: { name: string; path: string }[];
  handleDeleteTopic: (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    _: string
  ) => void;
  handleMoveToSection: (
    e: React.MouseEvent<HTMLDivElement, globalThis.MouseEvent>,
    _: string
  ) => void;
}

export const Options: FC<IOptions> = ({
  sections,
  handleMoveToSection,
  handleDeleteTopic,
}) => {
  return (
    <Box
      sx={{
        backgroundColor: "black",
        border: "1px solid #18D2FF",
        borderRadius: 0.4,
        boxSizing: "border-box",
        textAlign: "left",
        minWidth: 120,
      }}
    >
      {sections.length > 1 && ( //Show move to option only when more sections are available
        <Box
          sx={{
            boxSizing: "border-box",
            minWidth: "150px",
            width: "auto",
            height: "40px",
            display: "Flex",
            flexDirection: "column",
            justifyContent: "center",
          }}
        >
          <Typography
            sx={{
              paddingLeft: 2,
            }}
          >
            Move to
          </Typography>
        </Box>
      )}
      {sections.map((_) => (
        <Option
          key={_.name}
          handleMoveToSection={handleMoveToSection}
          path={_.path}
          label={_.name}
        />
      ))}
      <Option
        path={""}
        handleMoveToSection={handleDeleteTopic}
        label="Delete"
      />
    </Box>
  );
};
