import { Box, Typography, Icon, useFormant } from "@formant/ui-sdk";
import { FC } from "react";
import { IConfiguration } from "types";

interface IHeaderProps {
  height?: number | null;
}

export const Header: FC<IHeaderProps> = ({ height }) => {
  const context = useFormant();
  const configuration = context.configuration as IConfiguration;
  return (
    <Box
      sx={{
        width: "100%",
        height: !!height ? `${height}px` : "auto",
        borderBottom: "1px solid black",
        display: "flex",
        position: "relative",
      }}
    >
      <Box
        sx={{
          minWidth: !!height ? "25%" : "50%",
          display: "flex",
          alignItems: "center",
          overflow: "hidden",
          width: "auto",
          padding: !!height ? 0 : 1,
        }}
      >
        <span
          style={{
            color: "#bac4e2",
            fontSize: !!configuration?.fontSize ? `${configuration?.fontSize}px` : "14px",
          }}
        >
          Stream
        </span>
      </Box>
      <Box
        sx={{
          minWidth: !!height ? "25%" : "50%",
          padding: !!height ? 0 : 1,
          display: "flex",
          alignItems: "center",
          overflow: "hidden",
        }}
      >
        <span
          style={{
            color: "#bac4e2",
            fontSize: !!configuration?.fontSize ? `${configuration?.fontSize}px` : "14px",
          }}
        >
          Value
        </span>
      </Box>
      {!!height && (
        <>
          <Box
            sx={{
              width: "25%",
              padding: !!height ? 0 : 1,
              display: "flex",
              alignItems: "center",
              overflow: "hidden",
            }}
          >
            <span
              style={{
                color: "#bac4e2",
                fontSize: !!height ? (height / 2 > 14 ? 12 : height / 2) : 14,
              }}
            >
              Stream
            </span>
          </Box>
          <Box
            sx={{
              width: "25%",
              padding: !!height ? 0 : 1,
              display: "flex",
              alignItems: "center",
              overflow: "hidden",
            }}
          >
            <span
              style={{
                color: "#bac4e2",
                fontSize: !!height ? (height / 2 > 14 ? 12 : height / 2) : 14,
              }}
            >
              Value
            </span>
          </Box>
        </>
      )}

      {/* <Box
        display={"flex"}
        alignItems="center"
        justifyContent="center"
        flexDirection={"column"}
        borderRadius={25}
        position="absolute"
        height={!!height ? height : 30}
        width={!!height ? height : 30}
        right={!!height ? 23 : 50}
        top={!!height ? 0 : 5}
        onClick={() => {
          location.reload();
        }}
        sx={{
          ":hover": {
            backgroundColor: "#3b4668",
            cursor: "pointer",
          },
        }}
      >
        <Icon
          sx={{
            height: !!height ? height / 1.5 : "auto",
          }}
          name="refresh"
        />
      </Box> */}
    </Box>
  );
};
