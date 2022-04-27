import {
  FormControl,
  InputLabel,
  Select as MuiSelect,
  SelectChangeEvent,
  SxProps,
  Theme,
} from "@mui/material";
import React from "react";
export interface ISelectProps {
  label: string;
  children: React.ReactNode;
  sx?: SxProps<Theme>;
  value?: any;
  onChange?: (value: any) => void;
}

export function Select(props: ISelectProps) {
  const onDidChange = (event: SelectChangeEvent<any>) => {
    if (props.onChange) props.onChange(event.target.value);
  };
  return (
    <FormControl sx={props.sx || { width: "100%" }}>
      <InputLabel>{props.label}</InputLabel>
      <MuiSelect value={props.value} label={props.label} onChange={onDidChange}>
        {props.children}
      </MuiSelect>
    </FormControl>
  );
}
