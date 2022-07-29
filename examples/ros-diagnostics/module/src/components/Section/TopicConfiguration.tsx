import { Box, TextField, Icon } from "@formant/ui-sdk";
export const TopicConfiguration = () => {
  return (
    <Box
      sx={{
        display: "flex",
        minWidth: 700,
        justifyContent: "space-between",
        alignItems: "center",
        marginBottom: 1.6,
        height: 57,
      }}
      display={"flex"}
    >
      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Topic"}
        variant="filled"
      />
      <TextField
        sx={{ marginRight: 1, width: "45%" }}
        type="text"
        label={"Type"}
        variant="filled"
      />
      <TextField
        sx={{ marginRight: 1, width: 100 }}
        type="text"
        label={"Hz"}
        variant="filled"
      />
      <Box
        sx={{
          height: 35,
          width: 40,
          borderRadius: 25,
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          backgroundColor: "#292f43",
          ":hover": {
            backgroundColor: "#3b4668",
            cursor: "pointer",
          },
        }}
      >
        <Icon name="more" />
      </Box>
    </Box>
  );
};
