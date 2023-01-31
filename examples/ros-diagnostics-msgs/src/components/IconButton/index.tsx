import { FC } from "react";
import styles from "./index.module.scss";
import { Icon } from "@formant/ui-sdk";
interface IiconButtonProps {
  onClick: () => void;
}

export const IconButton: FC<IiconButtonProps> = ({ onClick }) => {
  return (
    <div onClick={onClick} className={styles["icon-button"]}>
      <Icon sx={{ color: "white" }} name="arrow-left" />
    </div>
  );
};
