import React from "react";
import { darkTheme } from "../src/theme";
import { ThemeProvider, createTheme } from "@mui/material/styles";
import { ThemeProvider as Emotion10ThemeProvider } from "emotion-theming";
import { CssBaseline } from "@mui/material";

const theme = createTheme(darkTheme);

const withThemeProvider = (Story, context) => {
  return (
    <Emotion10ThemeProvider theme={theme}>
      <ThemeProvider theme={theme}>
        <CssBaseline>
          <Story {...context} />
        </CssBaseline>
      </ThemeProvider>
    </Emotion10ThemeProvider>
  );
};

export const decorators = [withThemeProvider];
