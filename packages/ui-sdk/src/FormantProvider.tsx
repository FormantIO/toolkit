import * as React from "react";

import ThemeProvider from "@mui/material/styles/ThemeProvider";
import CssBaseline from "@mui/material/CssBaseline";
import { darkTheme, lightTheme, defaultTheme } from "./theme";
import { createTheme } from "@mui/material/styles";
import { useContext } from "react";
import { useConfiguration } from "./hooks";
interface IFormantProviderProps {
  theme?: "dark" | "light";
  children: React.ReactNode;
  parseConfiguration?: boolean;
}

interface UseFormant {
  configuration: string | { [key: string]: any } | undefined;
}

export function useFormant(): UseFormant {
  return useContext(FormantContext);
}

export const FormantContext = React.createContext<UseFormant>(undefined!);

export function FormantProvider({
  theme,
  children,
  parseConfiguration,
}: IFormantProviderProps) {
  const configuration = useConfiguration({ parse: !!parseConfiguration });
  const muiTheme = createTheme(
    theme === "dark" ? darkTheme : theme === "light" ? lightTheme : defaultTheme
  );

  // We can put global caches for data here
  const useFormant = { configuration };

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
