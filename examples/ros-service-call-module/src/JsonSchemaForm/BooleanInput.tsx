import React, { ChangeEventHandler, FC, useCallback } from "react";
import { get } from "lodash";
import { Box, Typography, Switch } from "@formant/ui-sdk";

import { updatePath } from "./updatePath";
import { IInputProps, JsonBooleanSchema } from "./types";
import { capitalize } from "./captialize";

export const BooleanInput: FC<IInputProps<JsonBooleanSchema>> = (props) => {
  const { params, schema, setParams, path } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => {
      setParams((prev) => updatePath(prev, path, e.target.checked));
    },
    [path, setParams]
  );

  return (
    <Box display="flex" height={56} alignItems="center" marginBottom={2}>
      <Typography>
        {capitalize(schema.title)}
        {": "}
      </Typography>
      <Switch
        checked={get(params, path) ?? false}
        size="small"
        onChange={handleChange}
      />
    </Box>
  );
};
