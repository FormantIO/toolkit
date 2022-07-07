import { FC, useState } from "react";
import { TextField } from "@formant/ui-sdk";
import "./index.css";
interface IIntegerInputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
}

export const IntegerInput: FC<IIntegerInputProps> = ({
  jsonSchemaObject,
  currentStateObject,
  property,
}) => {
  const [temp, setTemp] = useState("");
  const [error, setError] = useState("");
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
      className="formant-integer-input"
      sx={{ marginBottom: "36px" }}
      fullWidth={true}
      value={
        currentStateObject[jsonSchemaObject.properties[property].title] ?? ""
      }
      onChange={(ev) => {
        if (!isValid(ev.target.value)) {
          return;
        }
        setError("");
        setTemp(ev.target.value);
        jsonSchemaObject.title in currentStateObject
          ? (currentStateObject[jsonSchemaObject.title] = {
              ...currentStateObject[jsonSchemaObject.title],
              [jsonSchemaObject.properties[property].title]: ev.target.value,
            })
          : (currentStateObject[jsonSchemaObject.properties[property].title] =
              ev.target.value);
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
