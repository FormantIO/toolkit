import React, { FC, useCallback, useMemo, useState } from "react";
import { TextField } from "../../main";
import { IInputProps, JsonArraySchema } from "./types";
import { get, isInteger } from "lodash";
import { updatePath } from "./updatePath";
import { capitalize } from "./capitalize";

export const ArrayInput: FC<IInputProps<JsonArraySchema>> = (props) => {
  const { params, schema, setParams, path } = props;
  const [error, setError] = useState("");

  const currentValue = useMemo(() => {
    return get(params, path) !== undefined ? get(params, path) : [""];
  }, [params, setParams]);

  const handleOnBlur = useCallback(
    (index: number) => {
      setError("");
      //Avoids deleting the first textfield
      if (currentValue.length === 2) return;

      if (currentValue[index].length > 0) {
        //if text field not empty save
        setParams((prev) => updatePath(prev, path, currentValue));
        return;
      }
      currentValue.pop();
      if (!(currentValue[index].length > 0)) {
        //deletes text field if is empty
        setParams((prev) =>
          updatePath(prev, path, [
            ...currentValue.slice(0, index),
            ...currentValue.slice(index + 1),
          ])
        );
        return;
      }
      setParams((prev) => updatePath(prev, path, currentValue));
    },
    [currentValue]
  );

  const handleOnFocus = useCallback(
    (index: number) => {
      //Adds a text field wehn cliked in the last text field
      if (index + 1 === currentValue.length) {
        currentValue.push("");
        setParams((prev) => updatePath(prev, path, currentValue));
      }
    },
    [currentValue]
  );

  const handleChange = useCallback(
    (e: any, index: number) => {
      const { value } = e.target;
      setError("");
      if (schema.items.type === "integer") {
        if (value === "") {
          currentValue[index] = e.target.value;
          setParams((prev) => updatePath(prev, path, currentValue));
          return;
        }
        const newestInput = value.at(-1)!;
        const letterToFloat = parseFloat(newestInput);
        if (isInteger(letterToFloat)) {
          currentValue[index] = e.target.value;
          setParams((prev) => updatePath(prev, path, currentValue));
          return;
        }
        setError("Please enter a valid integer");
        return;
      }
      currentValue[index] = e.target.value;
      setParams((prev) => updatePath(prev, path, currentValue));
    },
    [path, setParams]
  );

  return (
    <>
      {currentValue.map((_: any, index: number) => {
        return (
          <TextField
            className="formant-integer-input "
            key={index}
            type="text"
            sx={{ marginBottom: "16px" }}
            label={capitalize(schema.title)}
            fullWidth={true}
            onFocus={() => handleOnFocus(index)}
            onBlur={() => handleOnBlur(index)}
            onChange={(e) => handleChange(e, index)}
            value={currentValue[index] ?? ""}
            helperText={
              //Shows error if exist at the bottom of the last text input
              index + 1 === currentValue.length && error.length > 1 ? error : ""
            }
            variant="filled"
          />
        );
      })}
    </>
  );
};
