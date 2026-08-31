import { Box, Icon, Button, Typography } from "@formant/ui-sdk";
import { FC, useEffect, useState } from "react";

interface IHeaderProps {
  onBack: () => void;
  buttonLabel?: string;
  onClick?: () => void;
  label?: string;
}

export const Header: FC<IHeaderProps> = ({
  onBack,
  buttonLabel,
  onClick,
  label,
}) => {
  let headerTitle: HTMLElement;
  const handleHeader = () => {
    if (!headerTitle) return;
    if (window.scrollY > 1) {
      headerTitle.classList.add("fade-out");
    }
    if (window.scrollY === 0) {
      headerTitle.classList.remove("fade-out");
      headerTitle.classList.add("fade-in");
    }
  };

  useEffect(() => {
    headerTitle = document.getElementById("header-label")!;
    window.addEventListener("scroll", handleHeader);
    return () => {
      window.removeEventListener("scroll", handleHeader);
    };
  }, []);

  return (
    <Box
      height={"70px"}
      display={"flex"}
      alignItems="center"
      justifyContent="space-between"
      paddingLeft={3}
      paddingRight={3}
      width="100vw"
    >
      <Box display={"flex"} alignItems="center">
        <Typography
          id="header-label"
          sx={{
            color: "white",
          }}
          variant="h2"
        >
          {label}
        </Typography>
      </Box>

      <Button onClick={onClick} variant="contained" size="medium">
        CREATE Section
      </Button>
    </Box>
  );
};
