import { FC, useState, useCallback, ChangeEventHandler } from "react";
import { TextField } from "@formant/ui-sdk";
import { capitalize } from "./capitalize";
import { get, isInteger } from "lodash";
import { updatePath } from "./updatePath";
import { IInputProps, JsonIntegerSchema } from "./types";

export const IntegerInput: FC<IInputProps<JsonIntegerSchema>> = (props) => {
  const [error, setError] = useState("");
  const { params, path, setParams, schema } = props;

  const handleChange = useCallback<ChangeEventHandler<HTMLInputElement>>(
    (e) => {
      const { value } = e.target;
      setError("");
      if (value === "") {
        setParams((prev) => updatePath(prev, path, e.target.value));
        return;
      }

      const newestInput = value.at(-1)!;
      const letterToFloat = parseFloat(newestInput);
      isInteger(letterToFloat)
        ? setParams((prev) => updatePath(prev, path, e.target.value))
        : setError("Please enter a valid integer");
    },
    [path, setParams]
  );

  return (
    <TextField
      className="formant-integer-input "
      type="text"
      sx={{ marginBottom: "36px" }}
      fullWidth={true}
      value={get(params, path) ?? ""}
      onChange={handleChange}
      label={capitalize(schema.title)}
      helperText={error}
      variant="filled"
    />
  );
};
