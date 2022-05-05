import * as React from "react";
import { TextField } from "@formant/ui-sdk";
import { TextLayerField } from "../model/LayerField";

export interface IFieldEditor {
  fieldId: string;
  field: TextLayerField;
  value: string | undefined;
  onChange: (fieldId: string, value: string) => void;
}

export function FieldEditor({ fieldId, field, value, onChange }: IFieldEditor) {
  const [initialValue, setInitialValue] = React.useState<string | undefined>(
    value
  );
  const [currentValue, setCurrentValue] = React.useState<string | undefined>(
    value
  );

  if (initialValue !== value) {
    setInitialValue(value);
    setCurrentValue(value);
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
