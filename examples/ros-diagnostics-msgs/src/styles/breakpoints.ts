export const breakpoints = {
  small: "785px",
  medium: "768px",
  large: "992px",
  xlarge: "1200px",
};

export const mediaQueries = {
  small: `@media (max-width: ${breakpoints.small})`,
  medium: `@media (min-width: ${breakpoints.medium})`,
  large: `@media (min-width: ${breakpoints.large})`,
  xlarge: `@media (min-width: ${breakpoints.xlarge})`,
};
