import { Dispatch, SetStateAction } from "react";

export type ServiceParameters = { [key: string]: string | ServiceParameters };

export type ServiceParameterSetter = Dispatch<
  SetStateAction<ServiceParameters>
>;
