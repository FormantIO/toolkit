import { createTheme, Theme, ThemeOptions } from "@mui/material";

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
    common: {
      black: "#ffffff",
      white: "#000000",
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
      dark: "#7F5072",
    },
    warning: {
      main: "#a961e4",
      dark: "#564A94",
    },
    info: {
      main: "#20a0ff",
      dark: "#256FAF",
    },
    success: {
      main: "#2ec495",
      dark: "#2D8376",
    },
  },
};

function createComponents(theme: Theme) {
  const components: ThemeOptions = {
    components: {
      MuiCssBaseline: {
        styleOverrides: {
          body: {
            "&::-webkit-scrollbar, & *::-webkit-scrollbar": {
              background: "transparent",
              width: "0.5rem",
              height: "0.5rem",
            },
            "&::-webkit-scrollbar-thumb, & *::-webkit-scrollbar-thumb": {
              background: theme.palette.primary.light,
              borderRadius: "0.25rem",
            },
            "&::-webkit-scrollbar-thumb:hover, & *::-webkit-scrollbar-thumb:hover":
              {
                backgroundColor: theme.palette.primary.light,
              },
            "&::-webkit-scrollbar-corner, & *::-webkit-scrollbar-corner": {
              backgroundColor: theme.palette.primary.light,
            },
          },
        },
      },
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
          outlined: {
            borderRadius: 4,
            padding: "0.125rem 0.375rem",
            "&.Mui-focused": {
              backgroundColor: "#2d3855",
            },
          },
        },
      },
      MuiFilledInput: {
        styleOverrides: {
          root: {
            backgroundColor: theme.palette.primary.dark,
            borderTop: "1px solid transparent",
            ":before": {
              borderBottom: `1px dotted ${theme.palette.primary.light}`,
            },
            "&.Mui-focused": {
              background: theme.palette.common.black,
              border: `1px solid ${theme.palette.secondary.main}`,
              borderBottom: "none !important",
              borderTop: `1px solid ${theme.palette.secondary.main}`,
            },
            // ":after": {
            //   borderBottom: `1px solid ${theme.palette.secondary.main}`,
            // },
          },
        },
      },
      MuiOutlinedInput: {
        styleOverrides: {
          root: {
            backgroundColor: theme.palette.primary.dark,
            borderBottomLeftRadius: 0,
            borderBottomRightRadius: 0,
            borderBottom: `1px dotted ${theme.palette.primary.light}`,

            "&.Mui-focused": {
              border: `1px solid ${theme.palette.secondary.main}`,
              backgroundColor: theme.palette.common.black,
            },
          },
          notchedOutline: {
            border: "transparent",
          },

          input: {
            "&:-webkit-autofill": {
              "-webkit-box-shadow": `0 0 0 100px ${theme.palette.primary.dark} inset`,
              "-webkit-text-fill-color": theme.palette.primary.light,
            },
          },
        },
      },

      MuiFormHelperText: {
        styleOverrides: {
          root: {
            color: theme.palette.error.main,
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
            "&.Mui-disabled": {
              color: theme.palette.primary.main,
            },
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
            background: theme.palette.primary.light,
            color: theme.palette.common.black,
            "@media(hover: hover)": {
              "&:hover": {
                boxShadow: `0 0 0 0.4rem ${theme.palette.primary.light}`,
                background: theme.palette.primary.light,
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
          outlinedPrimary: {
            color: theme.palette.primary.light,
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
      MuiNativeSelect: {
        styleOverrides: {
          root: {
            backgroundColor: theme.palette.primary.dark,
            borderTop: "1px solid transparent",
          },
          icon: {
            color: theme.palette.primary.light,
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
      MuiSnackbarContent: {
        styleOverrides: {
          root: {
            backgroundColor: theme.palette.primary.dark,
            color: theme.palette.primary.light,
          },
        },
      },
      MuiSwitch: {
        styleOverrides: {
          switchBase: {
            "&.Mui-checked+.MuiSwitch-track": {
              backgroundColor: theme.palette.common.black,
            },

            "&.Mui-checked .MuiSwitch-thumb": {
              backgroundColor: theme.palette.secondary.main,
            },
          },
          thumb: {
            backgroundColor: theme.palette.primary.light,
          },
          track: {
            backgroundColor: theme.palette.common.black,
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
