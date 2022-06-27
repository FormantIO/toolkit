import React, { FC } from "react";
import { Typography } from "@formant/ui-sdk";
import { TextInput } from "./TextInput";
import { NumberInput } from "./NumberInput";
import { BooleanInput } from "./BooleanInput";
import { IntegerInput } from "./IntegerInput";
import { ArrayInput } from "./ArrayInput";

interface IJsonSchemaFormProps {
  jsonSchemaObject: any;
  currentStateObject: any;
}
export const JsonSchemaForm: FC<IJsonSchemaFormProps> = ({
  jsonSchemaObject,
  currentStateObject,
}) => {
  if (jsonSchemaObject === undefined || currentStateObject === undefined)
    return <></>;
  if (jsonSchemaObject.type === "string") return <></>;
  const _objectKeys = Object.keys(jsonSchemaObject.properties);
  return (
    <>
      {_objectKeys.map((_: any) => {
        if (jsonSchemaObject.properties[_].type === "array") {
          return (
            <ArrayInput
              key={jsonSchemaObject.properties[_].title}
              jsonSchemaObject={jsonSchemaObject}
              currentStateObject={currentStateObject}
              property={_}
              type={jsonSchemaObject.properties[_].items.type}
            />
          );
        }
        if (jsonSchemaObject.properties[_].type === "boolean") {
          return (
            <BooleanInput
              key={jsonSchemaObject.properties[_].title}
              jsonSchemaObject={jsonSchemaObject}
              currentStateObject={currentStateObject}
              property={_}
            />
          );
        }
        if (jsonSchemaObject.properties[_].type === "number") {
          return (
            <NumberInput
              key={jsonSchemaObject.properties[_].title}
              jsonSchemaObject={jsonSchemaObject}
              currentStateObject={currentStateObject}
              property={_}
            />
          );
        }
        if (jsonSchemaObject.properties[_].type === "integer") {
          return (
            <IntegerInput
              key={jsonSchemaObject.properties[_].title}
              jsonSchemaObject={jsonSchemaObject}
              currentStateObject={currentStateObject}
              property={_}
            />
          );
        }

        if (jsonSchemaObject.properties[_].type === "string") {
          return (
            <TextInput
              key={jsonSchemaObject.properties[_].title}
              jsonSchemaObject={jsonSchemaObject}
              currentStateObject={currentStateObject}
              property={_}
            />
          );
        }
        if (jsonSchemaObject.properties[_].type === "object") {
          currentStateObject[jsonSchemaObject.properties[_].title] = {};
          return (
            <React.Fragment key={jsonSchemaObject.properties[_].title}>
              <Typography variant="h3">
                {jsonSchemaObject.properties[_].title[0].toUpperCase() +
                  jsonSchemaObject.properties[_].title.slice(1)}
              </Typography>
              <div style={{ marginLeft: 10 }}>
                {
                  <JsonSchemaForm
                    jsonSchemaObject={jsonSchemaObject.properties[_]}
                    currentStateObject={
                      currentStateObject[jsonSchemaObject.properties[_].title]
                    }
                  />
                }
              </div>
            </React.Fragment>
          );
        }
      })}
    </>
  );
};
