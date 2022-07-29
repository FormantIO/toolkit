import { Button, Box } from "@formant/ui-sdk";
import { FC } from "react";

interface IFooterProps {
  onClick: () => void;
  label: string;
  disabled?: boolean;
  onCancel: () => void;
}

export const Footer: FC<IFooterProps> = ({
  onClick,
  label,
  onCancel,
  disabled,
}) => {
  return (
    <Box
      height={"80px"}
      padding={"20px"}
      boxShadow={"0 0 1.25rem #1c1e2d"}
      position={"fixed"}
      bottom={0}
      width={"100vw"}
      zIndex={20}
      sx={{
        backgroundColor: "#2d3855",
      }}
      textAlign="right"
    >
      <Button
        sx={{
          marginRight: 2,
        }}
        onClick={onCancel}
      >
        Cancel
      </Button>
      <Button
        onClick={onClick}
        variant="contained"
        size="large"
        color="secondary"
        // disabled={disabled}
      >
        {label}
      </Button>
    </Box>
  );
};
