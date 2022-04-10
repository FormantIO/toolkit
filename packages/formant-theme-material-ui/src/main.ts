import { ThemeOptions } from "@mui/material";

export const themeOptions: ThemeOptions = {
  palette: {
    mode: "dark",
    primary: {
      main: "#3b4668",
      dark: "#282f45",
      light: "#657197",
      contrastText: "#ffffff",
    },
    secondary: {
      main: "#18d2ff",
      light: "#18d2ff",
      dark: "#256faf",
    },
    background: {
      default: "#1c1e2d",
      paper: "#2d3855",
    },
    text: {
      primary: "#bac4e2",
    },
    error: {
      main: "#ea719d",
    },
    warning: {
      main: "#a961e4",
    },
    info: {
      main: "#20a0ff",
    },
    success: {
      main: "#2ec495",
    },
  },
  shape: {
    borderRadius: 30,
  },
  typography: {
    fontFamily: "'Source Sans Pro', monospace",
  },
};
