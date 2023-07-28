import { Icon, Box, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import styles from "./StatusHeader.module.scss";
import { StatusFilter, SeverityLevel } from "../types/types";

interface IStatusHeaderProps {
  filters: StatusFilter;
  handleFilter: (_: SeverityLevel) => void;
}

export const StatusHeader: FC<IStatusHeaderProps> = ({
  filters,
  handleFilter,
}) => {
  return (
    <Box
      sx={{
        display: "flex",
        width: "100%",
        height: 60,
        alignItems: "center",
        paddingLeft: 2,
        paddingRight: 2,
      }}
    >
      <Box
        onClick={() => {
          handleFilter("ok");
        }}
        className={styles.ok}
        sx={{
          width: 120,
          marginRight: 2,
          alignItems: "center",
          justifyContent: "center",
          paddingRight: 1,
          paddingLeft: 1,
          paddingTop: 0.5,
          paddingBottom: 0.5,
          borderRadius: 25,
          backgroundColor: filters["ok"] ? "#677194" : "transparent",
          display: "flex",
          ": hover": {
            cursor: "pointer",
          },
        }}
      >
        <Icon name="check" />
        <Typography sx={{ color: "white" }}>OK</Typography>
      </Box>
      <Box
        onClick={() => handleFilter("warning")}
        className={styles.warning}
        sx={{
          width: 120,
          marginRight: 2,
          alignItems: "center",
          justifyContent: "center",
          paddingRight: 1,
          paddingLeft: 1,
          paddingTop: 0.5,
          paddingBottom: 0.5,
          borderRadius: 25,
          backgroundColor: filters["warning"] ? "#677194" : "transparent",
          display: "flex",
          ": hover": {
            cursor: "pointer",
          },
        }}
      >
        <Icon name="warning" />
        <Typography sx={{ color: "white", marginLeft: 0.5 }}>
          Warning
        </Typography>
      </Box>
      <Box
        onClick={() => handleFilter("critical")}
        className={styles.error}
        sx={{
          width: 120,
          marginRight: 2,
          alignItems: "center",
          justifyContent: "center",
          paddingRight: 1,
          paddingLeft: 1,
          paddingTop: 0.5,
          paddingBottom: 0.5,
          borderRadius: 25,
          backgroundColor: filters["critical"] ? "#677194" : "transparent",
          display: "flex",
          ": hover": {
            cursor: "pointer",
          },
        }}
      >
        <Icon name="critical" />
        <Typography sx={{ color: "white", marginLeft: 0.5 }}>Error</Typography>
      </Box>
      <Box
        onClick={() => handleFilter("stale")}
        className={styles.stale}
        sx={{
          width: 120,
          marginRight: 2,
          alignItems: "center",
          justifyContent: "center",
          paddingRight: 1,
          paddingLeft: 1,
          paddingTop: 0.5,
          paddingBottom: 0.5,
          borderRadius: 25,
          backgroundColor: filters["stale"] ? "#677194" : "transparent",
          display: "flex",
          ": hover": {
            cursor: "pointer",
          },
        }}
      >
        {/* <Icon name="check" /> */}
        <Typography sx={{ color: "white" }}>Stale</Typography>
      </Box>
    </Box>
  );
};
