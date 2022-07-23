import { Box, Typography, Icon } from "@formant/ui-sdk";
import { FC } from "react";

interface IHeaderInterfaceProps {
  setShow: () => void;
}

export const Header: FC<IHeaderInterfaceProps> = ({ setShow }) => {
  return (
    <Box
      sx={{
        width: "100%",
        heigh: "auto",
        borderBottom: "1px solid black",
        display: "flex",
        position: "relative",
      }}
    >
      <Box
        sx={{
          width: "30%",
          padding: 1,
        }}
      >
        <Typography variant="body2" sx={{ color: "#bac4e2", fontSize: 14 }}>
          Stream Name
        </Typography>
      </Box>
      <Box
        sx={{
          padding: 1,
        }}
      >
        <Typography variant="body2" sx={{ color: "#bac4e2", fontSize: 14 }}>
          Last known Value
        </Typography>
      </Box>
      <Box
        display={"flex"}
        alignItems="center"
        justifyContent="center"
        flexDirection={"column"}
        borderRadius={25}
        position="absolute"
        height={30}
        width={30}
        right={10}
        top={5}
        onClick={setShow}
        sx={{
          ":hover": {
            backgroundColor: "#3b4668",
            cursor: "pointer",
          },
        }}
      >
        <Icon name="settings" />
      </Box>
    </Box>
  );
};
