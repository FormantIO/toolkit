import { ThemeOptions } from "@mui/material";
import { deepmerge } from "@mui/utils";
const commonTheme: ThemeOptions = {
  components: {
    MuiDialogActions: {
      styleOverrides: {
        root: {
          padding: "1.5rem",
        },
      },
    },
    MuiTooltip: {
      styleOverrides: {
        tooltip: {
          backgroundColor: "rgba(0, 0, 0, 0.87)",
          color: "white",
          padding: ".6rem",
        },
      },
    },
    MuiDialogTitle: {
      styleOverrides: {
        root: {
          // h3
          fontStyle: "normal",
          fontWeight: 500,
          fontSize: "1.125rem",
          lineHeight: "1.688rem",
          letterSpacing: "0.069rem",
        },
      },
    },
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
          height: "1.25rem",
          fontWeight: 500,
          fontSize: "0.688rem",
          lineHeight: "0.825rem",
          letterSpacing: "0.047rem",
        },
        sizeMedium: {
          height: "1.813rem",
          fontWeight: 500,
        },
        sizeLarge: {
          height: "2.563rem",
          fontWeight: 500,
          fontSize: "0.875rem",
          lineHeight: "1.05rem",
          letterSpacing: "0.053rem",
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
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "2rem",
      lineHeight: "2.5rem",
      letterSpacing: 0,
    },
    h2: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1.5rem",
      lineHeight: "2.125rem",
      letterSpacing: "0.069rem",
      fontFeatureSettings: `"zero" on`,
    },
    h3: {
      fontStyle: "normal",
      fontWeight: 500,
      fontSize: "1.125rem",
      lineHeight: "1.688rem",
      letterSpacing: "0.069rem",
    },
    h4: {
      fontStyle: "normal",
      fontWeight: 700,
      fontSize: "0.813rem",
      lineHeight: "1.375rem",
      letterSpacing: "0.069rem",
      textTransform: "uppercase",
    },
    h5: {
      fontStyle: "normal",
      fontWeight: 700,
      fontSize: "0.813rem",
      lineHeight: "1.125rem",
      letterSpacing: "0.038rem",
      textTransform: "uppercase",
    },
    h6: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "0.625rem",
      lineHeight: "0.938rem",
      letterSpacing: "0.063rem",
    },
    body1: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1rem",
      lineHeight: "1.688rem",
      letterSpacing: "0.063rem",
      fontFeatureSettings: `"zero" on`,
    },
    body2: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1rem",
      lineHeight: "1.688rem",
      letterSpacing: "0.063rem",
      fontFeatureSettings: `"zero" on`,
      fontVariantNumeric: "tabular-nums",
    },
    button: {
      fontWeight: 500,
      fontSize: "0.875rem",
      lineHeight: "1.05rem",
      letterSpacing: "0.053.rem",
      textTransform: "uppercase",
    },
  },
};

const darkComponents: ThemeOptions = {
  components: {
    MuiDialog: {
      styleOverrides: {
        paper: {
          background: "#2d3855",
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        containedPrimary: {
          "@media(hover: hover)": {
            "&:hover": {
              boxShadow: "0 0 0 0.4rem #657197",
              background: "#657197",
            },
          },
        },
        containedSecondary: {
          "@media(hover: hover)": {
            "&:hover": {
              boxShadow: "0 0 0 0.4rem #18d2ff",
              background: "#18d2ff",
            },
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
      default: "#2d3855",
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
          "@media(hover: hover)": {
            "&:hover": {
              boxShadow: "0 0 0 0.4rem #657197",
              background: "#657197",
            },
          },
        },
        containedSecondary: {
          "@media(hover: hover)": {
            "&:hover": {
              boxShadow: "0 0 0 0.4rem #3babff",
              background: "#3babff",
            },
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
