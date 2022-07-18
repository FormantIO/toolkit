import { FC } from "react";
import { Typography } from "@formant/ui-sdk";
import { TextInput } from "./TextInput";
import { NumberInput } from "./NumberInput";
import { BooleanInput } from "./BooleanInput";
import { IntegerInput } from "./IntegerInput";
import { ArrayInput } from "./ArrayInput";

import { IInputProps } from "./types";
import { capitalize } from "./capitalize";

export const JsonSchemaForm: FC<IInputProps> = ({
  schema,
  params,
  setParams,
  path,
}) => {
  switch (schema.type) {
    case "string": {
      return (
        <TextInput
          params={params}
          path={path}
          schema={schema}
          setParams={setParams}
        />
      );
    }
    case "number": {
      return (
        <NumberInput
          params={params}
          path={path}
          schema={schema}
          setParams={setParams}
        />
      );
    }
    case "boolean": {
      return (
        <BooleanInput
          params={params}
          path={path}
          schema={schema}
          setParams={setParams}
        />
      );
    }
    case "integer": {
      return (
        <IntegerInput
          params={params}
          path={path}
          schema={schema}
          setParams={setParams}
        />
      );
    }
    case "array": {
      return <>TODO {schema.type}</>; // TODO
    }
    case "object": {
      const { properties } = schema;
      return (
        <>
          <Typography variant="h3">{capitalize(schema.title)}</Typography>
          <div style={{ marginLeft: 10 }}>
            {Object.keys(properties).map((key) => {
              const childSchema = properties[key];
              const nextPath = path ? [...path, key] : [key];
              return (
                <JsonSchemaForm
                  key={nextPath.join(".")}
                  schema={childSchema}
                  path={nextPath}
                  params={params}
                  setParams={setParams}
                />
              );
            })}
          </div>
        </>
      );
    }
    default:
      return <>Unsupported schema.type</>;
  }
};
