import * as React from "react";
import { TextField } from "@formant/ui-sdk";
import { LayerField, LayerFieldType } from "../model/LayerField";

export interface IFieldEditor {
  fieldId: string;
  field: LayerField;
  value: LayerFieldType | undefined;
  onChange: (fieldId: string, value: LayerFieldType) => void;
}

export function FieldEditor({ fieldId, field, value, onChange }: IFieldEditor) {
  const [initialValue, setInitialValue] = React.useState<
    LayerFieldType | undefined
  >(value);
  const [currentValue, setCurrentValue] = React.useState<
    LayerFieldType | undefined
  >(value);

  if (initialValue !== value) {
    setInitialValue(value);
    setCurrentValue(value);
  }
  if (field.type !== "string") {
    return <div>"TODO!"</div>;
  }

  return (
    <TextField
      label={field.name}
      value={currentValue}
      placeholder={field.placeholder}
      onChange={(e) => {
        const newValue = e.target.value || "";
        setCurrentValue(newValue);
        onChange(fieldId, newValue);
      }}
    />
  );
}
