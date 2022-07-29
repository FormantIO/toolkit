import { Box, Typography, TextField, Icon } from "@formant/ui-sdk";
import { TopicConfiguration } from "./TopicConfiguration";
export const Section = () => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        justifyContent: "left",
        textAlign: "left",
      }}
    >
      <Box
        sx={{
          display: "flex",
          width: "100%",
          alignItems: "center",
          justifyContent: "space-between",
        }}
      >
        <TextField
          type="text"
          sx={{ marginBottom: "16px", width: 230 }}
          label={"Section"}
          variant="standard"
          value={"Navigation"}
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
              backgroundColor: "#292f43",
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
              backgroundColor: "#292f43",
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
      <TopicConfiguration />
      <TopicConfiguration />
      <TopicConfiguration />
      <TextField
        type="text"
        sx={{ marginBottom: "16px", width: 230, margiTop: 20 }}
        label={"Section"}
        variant="standard"
        value={"Default"}
      />

      <TopicConfiguration />
      <TopicConfiguration />
      <TopicConfiguration />
    </Box>
  );
};
