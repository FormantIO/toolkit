import { Typography } from "@formant/ui-sdk";
import { FC } from "react";
import styles from "./index.module.scss";
import { IconButton } from "../IconButton";
import "../../index.css";
interface IDiagnosticsTableProps {
  diagnosticsDetails: any[] | null;
  active: string | null;
}

export const DiagnosticsTable: FC<IDiagnosticsTableProps> = ({
  diagnosticsDetails,
  active,
}) => {
  const handleBack = () => {
    const table = document.getElementById("diagnostics-table");
    table?.classList.remove("slide-out-left");
    table?.classList.add("slide-out-right");
  };

  return !!diagnosticsDetails && diagnosticsDetails.length > 0 ? (
    <div id="diagnostics-table" className={styles["diagnostic-table"]}>
      <div className={styles["diagnostic-table-header"]}>
        <IconButton onClick={handleBack} />
        <Typography sx={{ color: "white" }}>{active}</Typography>
      </div>
      {diagnosticsDetails.map((_) => {
        return (
          <div key={_.key} className={styles["diagnostic-table-row"]}>
            <Typography
              sx={{
                minWidth: "400px",
                padding: 1,
                color: "white",
              }}
            >
              {_.key}
            </Typography>
            <Typography sx={{ padding: 1, color: "white" }}>
              {_.value}
            </Typography>
          </div>
        );
      })}
    </div>
  ) : null;
};
