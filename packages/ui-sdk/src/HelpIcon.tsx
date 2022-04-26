import { Tooltip } from "@mui/material";
import React from "react";
import { Icon } from "./Icon";

export interface IHelpIconProps {
  description:
    | boolean
    | React.ReactChild
    | React.ReactFragment
    | React.ReactPortal;
}

export function HelpIcon(props: IHelpIconProps) {
  return (
    <Tooltip title={props.description}>
      <div>
        <Icon name="help" />
      </div>
    </Tooltip>
  );
}
