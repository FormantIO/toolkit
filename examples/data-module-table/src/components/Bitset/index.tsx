import { Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import "./index.css";

interface IBitsetProps {
  name: string;
  value: { keys: string[]; values: boolean[] };
}

export const Bitset: FC<IBitsetProps> = ({ name, value }) => {
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
        {value.keys.map((_, idx) => (
          <Box>
            <Typography
              sx={{
                borderBottom: "1px solid black",
                padding: 1,
                minWidth: 200,
                color: "white",
              }}
              variant="body1"
            >{`${_}: ${value.values[idx]}`}</Typography>
          </Box>
        ))}
      </Box>
    </Box>
  );
};
