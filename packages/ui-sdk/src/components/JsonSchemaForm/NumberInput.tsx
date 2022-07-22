import React, { FC, ChangeEventHandler, useCallback } from "react";
import { TextField } from "../../main";
import { capitalize } from "./capitalize";
import { updatePath } from "./updatePath";
import { JsonNumberSchema } from "./types";
import { IInputProps } from "./types";
import { get } from "lodash";
import { ServiceParameters } from "./ServiceParameters";

export const NumberInput: FC<IInputProps<JsonNumberSchema>> = (props) => {
  const { path, params, setParams, schema } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) =>
      setParams((prev: ServiceParameters) =>
        updatePath(prev, path, e.target.value)
      ),
    [path, setParams]
  );

  return (
    <TextField
      type="number"
      sx={{ marginBottom: "16px" }}
      fullWidth={true}
      value={get(params, path) ?? ""}
      onChange={handleChange}
      label={capitalize(schema.title)}
      variant="filled"
    />
  );
};
