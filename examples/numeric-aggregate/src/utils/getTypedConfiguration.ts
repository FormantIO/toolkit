import { INumericConfiguration, INumericSetConfiguration } from "../types";

export const isNumericConfiguration = (
  _: INumericConfiguration | INumericSetConfiguration
): _ is INumericConfiguration =>
  (_ as INumericConfiguration).numericStream !== undefined &&
  (_ as INumericConfiguration).numericStream.length > 0;

export const isNumericSetConfiguration = (
  _: INumericConfiguration | INumericSetConfiguration
): _ is INumericSetConfiguration =>
  (_ as INumericSetConfiguration).numericSetStream !== undefined &&
  (_ as INumericSetConfiguration).numericSetStream.length > 0;

export const getTypedConfiguration = (
  _: INumericConfiguration | INumericSetConfiguration
) => (isNumericConfiguration(_) ? _ : _);
