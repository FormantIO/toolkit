import { SxProps, Theme, Tooltip } from "@mui/material";
import React from "react";
import { Icon } from "./Icon";

export interface IHelpIconProps {
  description:
    | boolean
    | React.ReactChild
    | React.ReactFragment
    | React.ReactPortal;
  sx?: SxProps<Theme>;
}

export function HelpIcon(props: IHelpIconProps) {
  return (
    <Tooltip sx={props.sx} title={props.description}>
      <div>
        <Icon name="help" />
      </div>
    </Tooltip>
  );
}
