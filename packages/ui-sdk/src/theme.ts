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
          fontSize: ".65rem",
          lineHeight: "1.17rem",
          letterSpacing: "0.0715rem",
        },
        sizeMedium: {
          height: 29,
          fontWeight: 500,
        },
        sizeLarge: {
          height: 41,
          fontWeight: 500,
          padding: "1rem 3rem",
          fontSize: "1.05rem",
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
      fontSize: "1.82rem",
      lineHeight: "2.6rem",
      letterSpacing: 0,
    },
    h2: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1.3rem",
      lineHeight: "1.95rem",
      letterSpacing: "0.0715rem",
      fontFeatureSettings: `"zero" on`,
    },
    h3: {
      fontStyle: "normal",
      fontWeight: 500,
      fontSize: "1.17rem",
      lineHeight: "1.755rem",
      letterSpacing: "0.0715rem",
    },
    h4: {
      fontStyle: "normal",
      fontWeight: 700,
      fontSize: ".845rem",
      lineHeight: "1.43rem",
      letterSpacing: "0.0715rem",
      textTransform: "uppercase",
    },
    h5: {
      fontStyle: "normal",
      fontWeight: 700,
      fontSize: ".65rem",
      lineHeight: "1.17rem",
      letterSpacing: "0.0715rem",
      textTransform: "uppercase",
    },
    h6: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: ".65rem",
      lineHeight: ".975rem",
      letterSpacing: "0.065rem",
    },
    body1: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1.04rem",
      lineHeight: "1.755rem",
      letterSpacing: "0.0585rem",
      fontFeatureSettings: `"zero" on`,
    },
    body2: {
      fontStyle: "normal",
      fontWeight: "normal",
      fontSize: "1.04rem",
      lineHeight: "1.755rem",
      letterSpacing: "0.0585rem",
      fontFeatureSettings: `"zero" on`,
      fontVariantNumeric: "tabular-nums",
    },
    button: {
      fontSize: ".91rem",
      fontWeight: 500,
      lineHeight: "1.105rem",
      letterSpacing: ".0.05525rem",
      textTransform: "uppercase",
    },
  },
};

const darkComponents: ThemeOptions = {
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
