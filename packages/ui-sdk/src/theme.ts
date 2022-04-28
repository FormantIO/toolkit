import { createTheme, Theme, ThemeOptions } from "@mui/material";
import { Primary } from "./stories/Button.stories";

const baseTheme = createTheme({
  typography: {
    fontFamily: "'Moderat', 'Inter', 'Source Sans Pro', sans-serif",
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
});

const lightPalette: ThemeOptions = {
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

const darkPalette: ThemeOptions = {
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
    common: {
      black: "#000000",
      white: "#ffffff",
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

function createComponents(theme: Theme) {
  const components: ThemeOptions = {
    components: {
      MuiLink: {
        styleOverrides: {
          root: {
            color: theme.palette.secondary.main,
          },
        },
      },
      MuiDialogActions: {
        styleOverrides: {
          root: {
            padding: baseTheme.spacing(3),
          },
        },
      },
      MuiInputLabel: {
        styleOverrides: {
          root: {
            color: theme.palette.primary.light,
            "&.Mui-focused": {
              color: theme.palette.secondary.main,
            },
          },
        },
      },
      MuiFilledInput: {
        styleOverrides: {
          root: {
            backgroundColor: theme.palette.primary.dark,
            ":before": {
              borderBottom: `1px dotted ${theme.palette.primary.light}`,
            },
            "&.Mui-focused": {
              background: theme.palette.common.black,
              border: `1px solid ${theme.palette.secondary.main}`,
              borderBottom: "none",
            },
            ":after": {
              borderBottom: `1px solid ${theme.palette.secondary.main}`,
            },
          },
        },
      },
      MuiTooltip: {
        styleOverrides: {
          tooltip: {
            backgroundColor: "rgba(0, 0, 0, 0.87)",
            color: "white",
            padding: baseTheme.spacing(1.2),
          },
        },
      },
      MuiDialogTitle: {
        styleOverrides: {
          root: {
            color: theme.palette.common.white,
            fontStyle: "normal",
            fontWeight: 500,
            ...baseTheme.typography.h3,
          },
        },
      },
      MuiDialogContentText: {
        styleOverrides: {
          root: {
            color: theme.palette.primary.light,
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
            willChange: "background-color , color , border-color , box-shadow",
            transition:
              "background-color 200ms ease-in-out 0ms , color 200ms ease-in-out 0ms , border-color 200ms ease-in-out 0ms , box-shadow 200ms ease-in-out 0ms",
            borderRadius: 40,
            boxShadow: `0 0 0 0rem ${theme.palette.primary.main}`,
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
          containedPrimary: {
            color: theme.palette.common.white,
            "@media(hover: hover)": {
              "&:hover": {
                boxShadow: `0 0 0 0.4rem ${theme.palette.primary.main}`,
                background: theme.palette.primary.main,
              },
            },
          },
          containedSecondary: {
            color: theme.palette.common.black,
            "@media(hover: hover)": {
              "&:hover": {
                boxShadow: `0 0 0 0.4rem ${theme.palette.secondary.main}`,
                background: theme.palette.secondary.main,
              },
            },
          },
        },
      },
      MuiDialog: {
        styleOverrides: {
          paper: {
            background: theme.palette.background.paper,
          },
        },
      },
      MuiSelect: {
        styleOverrides: {
          icon: {
            color: theme.palette.primary.light,
          },
        },
      },
    },
  };
  return createTheme(theme, components);
}

export const darkTheme = createComponents(createTheme(baseTheme, darkPalette));

export const lightTheme = createComponents(
  createTheme(baseTheme, lightPalette)
);

export const defaultTheme = darkTheme;
