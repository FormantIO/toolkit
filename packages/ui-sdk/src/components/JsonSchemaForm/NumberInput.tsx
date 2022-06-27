import React, { FC } from "react";
import { TextField } from "../../main";

interface INumberInputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
}

export const NumberInput: FC<INumberInputProps> = ({
  jsonSchemaObject,
  currentStateObject,
  property,
}) => {
  return (
    <TextField
      type="number"
      key={jsonSchemaObject.properties[property].title}
      sx={{ marginBottom: "16px" }}
      fullWidth={true}
      value={currentStateObject[jsonSchemaObject.properties[property].title]}
      onChange={(ev) => {
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
      variant="filled"
    />
  );
};
