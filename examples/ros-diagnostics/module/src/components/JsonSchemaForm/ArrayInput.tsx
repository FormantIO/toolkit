import React, { FC, useEffect, useState } from "react";
import { TextField } from "@formant/ui-sdk";

interface IArrayInputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
  type: string;
}

export const ArrayInput: FC<IArrayInputProps> = ({
  jsonSchemaObject,
  currentStateObject,
  property,
  type,
}) => {
  const [array, setArray] = useState<(string | number)[]>([""]);
  // const [error, setError] = useState<string>("");

  useEffect(() => {
    currentStateObject[jsonSchemaObject.properties[property].title] =
      array.slice(0, array.length - 1);
  }, [array]);

  const handleOnBlur = (index: number) => {
    if (array.length === 1) return;
    if (array.length === index + 1) {
      array.pop();
      setArray([...array]);
    }
    if (array[index] === "") {
      setArray([...array.slice(0, index), ...array.slice(index + 1)]);
      return;
    }
  };

  const isValid = (_: string) => {
    if (_ === "") return true;

    if (!!_) {
      const newestInput = _.at(-1);
      const letterToFloat = parseFloat(newestInput!);
      if (Number.isInteger(letterToFloat)) return true;
    }
    // setError("Please enter a valid integer");
    return false;
  };

  const handleOnFocus = (index: number) => {
    if (index + 1 < array.length) {
      return;
    }
    array.push("");
    setArray([...array]);
  };

  const handleOnChange = (
    ev: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>,
    index: number
  ) => {
    const { value } = ev.target;
    if (type === "integer") {
      if (!isValid(value)) {
        return;
      }
    }
    array[index] = value;
    setArray([...array]);
  };

  return (
    <>
      {array.map((_: any, index: number) => {
        return (
          <TextField
            type="text"
            key={index}
            sx={{ marginBottom: "16px" }}
            fullWidth={true}
            onFocus={() => handleOnFocus(index)}
            value={array[index]}
            onBlur={() => handleOnBlur(index)}
            onChange={(ev) => handleOnChange(ev, index)}
            label={
              jsonSchemaObject.properties[property].title[0].toUpperCase() +
              jsonSchemaObject.properties[property].title.slice(1)
            }
            variant="filled"
          />
        );
      })}
    </>
  );
};
