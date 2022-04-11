import { ThemeOptions } from "@mui/material";
const commonTheme: ThemeOptions = {
  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: 40,
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
export const darkTheme: ThemeOptions = {
  ...commonTheme,
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
};

export const lightTheme: ThemeOptions = {
  ...commonTheme,
  palette: {
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
      main: "#e4e7f3",
      light: "#f1f3f9",
      dark: "#d6dcee",
    },
    secondary: {
      main: "#3babff",
      dark: "#76a7dc",
    },
  },
};

export const defaultTheme = darkTheme;
