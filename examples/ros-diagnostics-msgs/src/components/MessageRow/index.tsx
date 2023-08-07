import { Icon, Typography } from "@formant/ui-sdk";
import { FC } from "react";
import styles from "./index.module.scss";
import { SeverityLevel } from "../../types/types";
import styled from "@emotion/styled";
interface IMesageRowProps {
  message: string;
  active: string | null;
  setActive: () => void;
  level: number | string;
}

export const MessageRow: FC<IMesageRowProps> = ({
  message,
  setActive,
  active,
  level,
}) => {
  return (
    <Row onClick={setActive}>
      <div
        className={`${styles["inner-message"]} ${
          active === message ? styles["inner-message-active"] : ""
        }`}
      >
        <div className={styles.text}>
          <Icon
            name={
              level === 0 || level === "b'\\x00'"
                ? "check"
                : level === 1 || level === "b'\\x01'"
                ? "warning"
                : level === 2 || level === "b'\\x02'"
                ? "critical"
                : "online"
            }
          />
          <Typography
            sx={{
              color: "white",
              width: "auto",
            }}
          >
            {message}
          </Typography>
        </div>
        <div className={styles.chevron}>
          <Icon name="chevron-right" />
        </div>
      </div>
    </Row>
  );
};

const Row = styled.button`
  all: unset;
  box-sizing: border-box;
  height: 56px;
  display: flex;
  align-items: center;
  padding: 0 10px 0;
  width: auto;
  min-width: 400px;
  justify-content: space-between;
  &:hover {
    cursor: pointer;
  }
  p {
    margin-left: 8px;
  }
`;
