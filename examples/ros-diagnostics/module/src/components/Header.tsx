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
      position={"fixed"}
      top={0}
      left={0}
      height={"70px"}
      display={"flex"}
      alignItems="center"
      justifyContent="space-between"
      paddingLeft={2}
      paddingRight={2}
      borderRadius={25}
      width="100vw"
    >
      <Box display={"flex"} alignItems="center">
        <Box
          height={"40px"}
          width="40px"
          borderRadius={25}
          display={"flex"}
          alignItems="center"
          justifyContent={"center"}
          marginRight={1.5}
          onClick={onBack}
          sx={{
            ":hover": {
              backgroundColor: "#3b4668",
              cursor: "pointer",
            },
          }}
        >
          <Icon name="arrow-left" />
        </Box>
        {label && (
          <Typography
            id="header-label"
            sx={{
              color: "white",
              transitionTimingFunction: "ease-in",
              transition: ".5s",
            }}
            variant="h2"
          >
            {label}
          </Typography>
        )}
      </Box>
      {buttonLabel && (
        <Button onClick={onClick} variant="contained" size="medium">
          {buttonLabel}
        </Button>
      )}
    </Box>
  );
};
