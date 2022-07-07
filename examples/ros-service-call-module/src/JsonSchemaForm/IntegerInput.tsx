import { FC, useState, useEffect } from "react";
import { TextField } from "@formant/ui-sdk";

interface IIntegerInputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
  defaultValue: string;
}

export const IntegerInput: FC<IIntegerInputProps> = ({
  jsonSchemaObject,
  currentStateObject,
  property,
  defaultValue,
}) => {
  const [currentValue, setCurrentValue] = useState<number | string>();

  const [error, setError] = useState("");

  useEffect(() => {
    if (isNaN(parseInt(defaultValue))) return;
    setCurrentValue(parseInt(defaultValue));
  }, []);
  const isValid = (_: string) => {
    if (_ === "") return true;
    if (!!_) {
      const newestInput = _.at(-1);
      const letterToFloat = parseFloat(newestInput!);
      if (Number.isInteger(letterToFloat)) return true;
    }
    setError("Please enter a valid integer");
    return false;
  };

  return (
    <TextField
      type="phone"
      key={jsonSchemaObject.properties[property].title}
      sx={{ marginBottom: "36px" }}
      fullWidth={true}
      value={currentValue}
      onChange={(ev) => {
        if (!isValid(ev.target.value)) {
          return;
        }
        setError("");
        if (ev.target.value === "") {
          setCurrentValue("");
        } else {
          setCurrentValue(parseInt(ev.target.value));
        }
        currentStateObject[jsonSchemaObject.properties[property].title] =
          ev.target.value;
      }}
      label={
        jsonSchemaObject.properties[property].title[0].toUpperCase() +
        jsonSchemaObject.properties[property].title.slice(1)
      }
      helperText={error}
      variant="filled"
    />
  );
};
