import React, { ChangeEventHandler, FC, useCallback } from "react";
import { get } from "lodash";
import { TextField } from "@formant/ui-sdk";

import { updatePath } from "./updatePath";
import { IInputProps, JsonStringSchema } from "./types";
import { capitalize } from "./captialize";

export const TextInput: FC<IInputProps<JsonStringSchema>> = (props) => {
  const { params, schema, setParams, path } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => setParams((prev) => updatePath(prev, path, e.target.value)),
    [path, setParams]
  );

  return (
    <TextField
      type="text"
      sx={{ marginBottom: "16px" }}
      fullWidth={true}
      value={get(params, path) ?? ""}
      onChange={handleChange}
      label={capitalize(schema.title)}
      variant="filled"
    />
  );
};
