import { useContext } from "react";
import { FormantContext } from "../FormantProvider";

export function useFormant() {
  return useContext(FormantContext);
}
