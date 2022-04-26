import {
  FormControl,
  InputLabel,
  Select as MuiSelect,
  SxProps,
  Theme,
} from "@mui/material";
import React from "react";

export interface ISelectProps {
  label: string;
  children: React.ReactNode;
  sx?: SxProps<Theme>;
  value?: any;
}

export function Select(props: ISelectProps) {
  return (
    <FormControl sx={props.sx || { width: "100%" }}>
      <InputLabel>{props.label}</InputLabel>
      <MuiSelect value={props.value} label={props.label}>
        {props.children}
      </MuiSelect>
    </FormControl>
  );
}
