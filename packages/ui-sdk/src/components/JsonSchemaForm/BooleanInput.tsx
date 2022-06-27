import { Box, Typography, Switch } from "../../main";
import React, { FC } from "react";
interface INumberInputProps {
  jsonSchemaObject: any;
  currentStateObject: any;
  property: string;
}

export const BooleanInput: FC<INumberInputProps> = ({
  jsonSchemaObject,
  currentStateObject,
  property,
}) => {
  return (
    <Box display="flex" height={56} alignItems="center" marginBottom={2}>
      <Typography>
        {jsonSchemaObject.properties[property].title[0].toUpperCase() +
          jsonSchemaObject.properties[property].title.slice(1) +
          ": "}
      </Typography>
      <Switch
        size="small"
        onChange={(ev) => {
          jsonSchemaObject.title in currentStateObject
            ? (currentStateObject[jsonSchemaObject.title] = {
                ...currentStateObject[jsonSchemaObject.title],
                [jsonSchemaObject.properties[property].title]: ev.target.value,
              })
            : (currentStateObject[jsonSchemaObject.properties[property].title] =
                ev.target.value);
        }}
      />
    </Box>
  );
};
