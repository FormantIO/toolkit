import React, { FC, ChangeEventHandler, useCallback } from "react";
import { TextField } from "@formant/ui-sdk";
import { updatePath } from "./updatePath";
import { IInputProps, JsonStringSchema } from "./types";
import { capitalize } from "./capitalize";
import { get } from "lodash";

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
      onChange={handleChange}
      value={get(params, path) ?? ""}
      label={capitalize(schema.title)}
      variant="filled"
    />
  );
};
