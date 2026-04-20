import { TextField } from "@formant/ui-sdk";
import React, { FC, useEffect, useLayoutEffect, useState } from "react";

interface ITextinputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
  defaultValue: string;
}

export const TextInput: FC<ITextinputProps> = ({
  defaultValue,
  jsonSchemaObject,
  currentStateObject,
  property,
}) => {
  const [currentValue, setCurrentValue] = useState("");

  useEffect(() => {
    setCurrentValue(defaultValue);
  }, []);

  return (
    <TextField
      type="text"
      key={jsonSchemaObject.properties[property].title}
      sx={{ marginBottom: "16px" }}
      fullWidth={true}
      value={currentValue}
      onChange={(ev: React.ChangeEvent<HTMLInputElement>) => {
        setCurrentValue(ev.target.value);
        currentStateObject[jsonSchemaObject.properties[property].title] =
          ev.target.value;
      }}
      label={
        jsonSchemaObject.properties[property].title[0].toUpperCase() +
        jsonSchemaObject.properties[property].title.slice(1)
      }
      variant="filled"
    />
  );
};
