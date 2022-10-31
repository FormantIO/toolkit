import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import "./index.css";

interface Numeric {
  value: number;
  label: string;
  unit?: string;
}

interface INumericSetProps {
  name: string;
  value: Numeric[];
}

export const NumericSet: FC<INumericSetProps> = ({ name, value }) => {
  return (
    <Box
      sx={{
        display: "flex",
        borderBottom: "1px solid black",
      }}
    >
      <Typography
        sx={{
          borderRight: "1px solid black",
          padding: 1,
          minWidth: 200,
          color: "white",
        }}
        variant="body1"
      >
        {name}
      </Typography>
      <Box
        className="numeric-set"
        sx={{
          displaey: "flex",
          flexDirection: "column",
          width: "100%",
        }}
      >
        {value.map((_) => (
          <Box>
            <Typography
              sx={{
                borderBottom: "1px solid black",
                padding: 1,
                minWidth: 200,
                color: "white",
              }}
              variant="body1"
            >{`${_.label}: ${_.value.toFixed(2)}  ${_.unit ?? ""}`}</Typography>
          </Box>
        ))}
      </Box>
    </Box>
  );
};
