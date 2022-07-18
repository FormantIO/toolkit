import React, { FC, ChangeEventHandler, useCallback } from "react";
import { Box, Typography, Switch } from "@formant/ui-sdk";
import { capitalize } from "./capitalize";
import { JsonBooleanSchema } from "./types";
import { IInputProps } from "./types";
import { updatePath } from "./updatePath";
import { get } from "lodash";

export const BooleanInput: FC<IInputProps<JsonBooleanSchema>> = (props) => {
  const { params, schema, setParams, path } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => setParams((prev) => updatePath(prev, path, e.target.checked)),
    [path, setParams]
  );

  return (
    <Box display="flex" height={56} alignItems="center" marginBottom={2}>
      <Typography>
        {capitalize(schema.title)}
        {": "}
      </Typography>
      <Switch
        size="small"
        checked={get(params, path) ?? false}
        onChange={handleChange}
      />
    </Box>
  );
};
