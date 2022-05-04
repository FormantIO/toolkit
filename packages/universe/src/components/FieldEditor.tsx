import * as React from "react";
import { TextField } from "@formant/ui-sdk";
import { TextLayerField } from "../layers/UniverseLayerContent";

export interface IFieldEditor {
  fieldId: string;
  field: TextLayerField;
  initialValue: string | undefined;
  onChange: (fieldId: string, value: string) => void;
}

export function FieldEditor({
  fieldId,
  field,
  initialValue,
  onChange,
}: IFieldEditor) {
  const [value, setValue] = React.useState<string | undefined>(initialValue);
  return (
    <TextField
      label={field.name}
      value={value}
      placeholder={field.placeholder}
      onChange={(e) => {
        const newValue = e.target.value || "";
        setValue(newValue);
        onChange(fieldId, newValue);
      }}
    />
  );
}
