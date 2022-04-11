import { ThemeOptions } from "@mui/material";
import { deepmerge } from "@mui/utils";
const commonTheme: ThemeOptions = {
  components: {
    MuiButtonBase: {
      defaultProps: {
        disableRipple: true,
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          transitionDuration: "200ms",
          transitionTimingFunction: "ease-in-out",
          borderRadius: 40,
        },
        sizeSmall: {
          height: 20,
          fontWeight: 700,
          fontSize: "10px",
          lineHeight: "18px",
          letterSpacing: "1.1px",
        },
        sizeMedium: {
          height: 29,
          fontWeight: 500,
          fontSize: "11px",
          lineHeight: "13px",
          letterSpacing: "0.75px",
        },
        sizeLarge: {
          height: 41,
          fontWeight: 500,
          fontSize: "14px",
          lineHeight: "17px",
          letterSpacing: "0.85px",
        },
      },
    },
    MuiTextField: {
      styleOverrides: {
        root: {
          borderRadius: 4,
        },
      },
    },
  },
  typography: {
    fontFamily: "'Moderat','Source Sans Pro', sans-serif",
    h1: {
      fontSize: "32px",
      fontWeight: 400,
      lineHeight: "40px",
    },
    h2: {
      fontSize: "24px",
      fontWeight: 400,
      lineHeight: "34px",
      letterSpacing: "1.1px",
    },
    h3: {
      fontSize: "18px",
      fontWeight: 500,
      lineHeight: "27px",
      letterSpacing: "1.1px",
    },
    h4: {
      fontSize: "13px",
      fontWeight: 700,
      lineHeight: "22px",
      letterSpacing: "1.1px",
      textTransform: "uppercase",
    },
    h5: {
      fontSize: "13px",
      fontWeight: 400,
      lineHeight: "20px",
      letterSpacing: "0.6px",
    },
    h6: {
      fontSize: "10px",
      fontWeight: 400,
      lineHeight: "15px",
      letterSpacing: "1px",
      textTransform: "uppercase",
    },
    body1: {
      fontSize: "16px",
      fontWeight: 400,
      lineHeight: "27px",
      letterSpacing: "1px",
    },
    body2: {
      fontSize: "16px",
      fontWeight: 400,
      lineHeight: "27px",
      letterSpacing: "1px",
    },
    button: {
      fontSize: "14px",
      fontWeight: 500,
      lineHeight: "17px",
      letterSpacing: "0.85px",
      textTransform: "uppercase",
    },
  },
};

const darkComponents: ThemeOptions = {
  components: {
    MuiButton: {
      styleOverrides: {
        containedPrimary: {
          "&:hover": {
            boxShadow: "0 0 0 0.4rem #3b4668",
            background: "#3b4668",
          },
        },
        containedSecondary: {
          "&:hover": {
            boxShadow: "0 0 0 0.4rem #18d2ff",
            background: "#18d2ff",
          },
        },
      },
    },
  },
};

export const darkTheme: ThemeOptions = {
  ...deepmerge(commonTheme, darkComponents),
  palette: {
    mode: "dark",
    primary: {
      main: "#657197",
      dark: "#3B4668",
      light: "#BAC4E2",
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
};

const lightComponents: ThemeOptions = {
  components: {
    MuiButton: {
      styleOverrides: {
        containedPrimary: {
          "&:hover": {
            boxShadow: "0 0 0 0.4rem #657197",
            background: "#657197",
          },
        },
        containedSecondary: {
          "&:hover": {
            boxShadow: "0 0 0 0.4rem #3babff",
            background: "#3babff",
          },
        },
      },
    },
  },
};

export const lightTheme: ThemeOptions = {
  ...deepmerge(commonTheme, lightComponents),
  palette: {
    mode: "light",
    success: {
      main: "#34dea9",
      dark: "#2d8376",
    },
    info: {
      main: "#297ceb",
      dark: "#256faf",
      light: "#3BABFF",
    },
    error: {
      main: "#fd76a7",
      dark: "#7f5072",
    },
    warning: {
      main: "#ffb179",
      dark: "#94645f",
    },
    primary: {
      main: "#657197",
      light: "#BAC4E2",
      dark: "#3B4668",
    },
    secondary: {
      main: "#3babff",
      dark: "#76a7dc",
    },
    background: {
      default: "#FFFFFF",
      paper: "#F1F3F9",
    },
  },
};

export const defaultTheme = darkTheme;
