import {
  FormControl,
  InputLabel,
  MenuItem,
  NativeSelect,
  Select as MuiSelect,
  SelectChangeEvent,
  SxProps,
  Theme,
} from "@mui/material";
import React from "react";
import { MobileView, BrowserView } from "react-device-detect";
import styled from "styled-components";
export interface ISelectProps<T> {
  label: string;
  sx?: SxProps<Theme>;
  value?: T;
  onChange?: (value: T) => void;
  fullWidth?: boolean;
  items?: { label: string; value: T }[];
}

export function Select<T extends string | number | readonly string[]>(
  props: ISelectProps<T>
) {
  const onDidChange = (event: SelectChangeEvent<T>) => {
    if (props.onChange) props.onChange(event.target.value as T);
  };
  const onDidNativeChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    if (props.onChange) props.onChange(event.target.value as T);
  };
  return (
    <>
      <MobileView>
        <FormControl
          sx={{ minWidth: "20rem", ...(props.sx || {}) }}
          fullWidth={props.fullWidth}
        >
          <NativeSelect value={props.value} onChange={onDidNativeChange}>
            {(props.items || []).map((item) => (
              <option key={item.label} value={item.value}>
                {item.label}
              </option>
            ))}
          </NativeSelect>
        </FormControl>
      </MobileView>
      <BrowserView>
        <FormControl
          sx={{ minWidth: "20rem", ...(props.sx || {}) }}
          fullWidth={props.fullWidth}
          variant="filled"
        >
          <InputLabel>{props.label}</InputLabel>
          <MuiSelect
            value={props.value}
            label={props.label}
            onChange={onDidChange}
          >
            {(props.items || []).map((item) => (
              <MenuItem key={item.label} value={item.value}>
                {item.label}
              </MenuItem>
            ))}
          </MuiSelect>
        </FormControl>
      </BrowserView>
    </>
  );
}
