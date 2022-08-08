import { Box, Icon, TextField } from "@formant/ui-sdk";
import { FC } from "react";

interface ISectionHeader {
  name: string;
}

export const SectionHeader: FC<ISectionHeader> = ({ name }) => {
  return (
    <Box
      sx={{
        display: "flex",
        width: "100%",
        alignItems: "center",
        justifyContent: "space-between",
        marginTop: 1,
      }}
    >
      <TextField
        type="text"
        sx={{ marginBottom: "16px", width: 230 }}
        label={"Section"}
        variant="standard"
        value={name}
      />

      <Box
        sx={{
          display: "flex",
        }}
      >
        <Box
          sx={{
            height: 40,
            width: 40,
            borderRadius: 25,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            marginRight: 1,
            ":hover": {
              backgroundColor: "#3b4668",
              cursor: "pointer",
            },
          }}
        >
          <Icon name="plus" />
        </Box>
        <Box
          sx={{
            height: 40,
            width: 40,
            borderRadius: 25,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            ":hover": {
              backgroundColor: "#3b4668",
              cursor: "pointer",
            },
          }}
        >
          <Icon name="delete" />
        </Box>
      </Box>
    </Box>
  );
};
