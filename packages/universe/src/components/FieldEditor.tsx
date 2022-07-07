import * as React from "react";
import { TextField } from "@formant/ui-sdk";
import {
  LayerField,
  LayerFieldType,
  LayerFieldTypeMap,
} from "../model/LayerField";

export interface IFieldEditor {
  fieldId: string;
  field: LayerField;
  value: LayerFieldTypeMap[LayerFieldType] | undefined;
  onChange: (fieldId: string, value: LayerFieldTypeMap[LayerFieldType]) => void;
}

export function FieldEditor({ fieldId, field, value, onChange }: IFieldEditor) {
  const [initialValue, setInitialValue] = React.useState<
    LayerFieldTypeMap[LayerFieldType] | undefined
  >(value);
  const [currentValue, setCurrentValue] = React.useState<
    LayerFieldTypeMap[LayerFieldType] | undefined
  >(value);

  if (initialValue !== value) {
    setInitialValue(value);
    setCurrentValue(value);
  }
  if (field.type !== "text") {
    return <div>"TODO!"</div>;
  }

  return (
    <TextField
      label={field.name}
      value={currentValue}
      placeholder={field.placeholder.toString()}
      onChange={(e) => {
        const newValue = e.target.value || "";
        setCurrentValue(newValue);
        onChange(fieldId, newValue);
      }}
    />
  );
}
