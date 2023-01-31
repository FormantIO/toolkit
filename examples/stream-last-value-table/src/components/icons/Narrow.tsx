import { FC } from "react";
import styles from "./index.module.scss";

interface IProps {
  active: boolean;
  handleOnNarrow: () => void;
}

export const Narrow: FC<IProps> = ({ active, handleOnNarrow }) => {
  return (
    <div
      onClick={handleOnNarrow}
      className={`${styles.icon} ${active ? styles.active : ""}`}
    >
      <svg
        width="16"
        height="16"
        viewBox="0 0 16 16"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        <path
          fillRule="evenodd"
          clipRule="evenodd"
          d="M13 2H3C2.44772 2 2 2.44772 2 3V13C2 13.5523 2.44772 14 3 14H13C13.5523 14 14 13.5523 14 13V3C14 2.44772 13.5523 2 13 2ZM3 1C1.89543 1 1 1.89543 1 3V13C1 14.1046 1.89543 15 3 15H13C14.1046 15 15 14.1046 15 13V3C15 1.89543 14.1046 1 13 1H3Z"
          fill="#BAC4E2"
        />
        <path d="M3 4C3 3.44772 3.44772 3 4 3H8V7H3V4Z" fill="#BAC4E2" />
      </svg>
    </div>
  );
};
