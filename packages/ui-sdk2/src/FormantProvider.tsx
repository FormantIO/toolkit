import * as React from "react";

import ThemeProvider from "@mui/material/styles/ThemeProvider";
import CssBaseline from "@mui/material/CssBaseline";
import { darkTheme, lightTheme } from "./theme";
import { createTheme } from "@mui/material/styles";
import { useContext } from "react";
interface IFormantProviderProps {
  theme?: "dark" | "light";
  children: React.ReactNode;
}

interface UseFormant {}

export function useFormant(): UseFormant {
  return useContext(FormantContext);
}

export const FormantContext = React.createContext<UseFormant>(undefined!);

export function FormantProvider({
  theme = "dark",
  children,
}: IFormantProviderProps) {
  const muiTheme = createTheme(
    theme === "dark" ? darkTheme : theme === "light" ? lightTheme : {}
  );

  // We can put global caches for data here
  const useFormant = {};

  return (
    <ThemeProvider theme={muiTheme}>
      <CssBaseline>
        <FormantContext.Provider value={useFormant}>
          {children}
        </FormantContext.Provider>
      </CssBaseline>
    </ThemeProvider>
  );
}
