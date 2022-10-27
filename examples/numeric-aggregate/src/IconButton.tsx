import { FC } from "react";
import styles from "./index.module.scss";

import { Icon } from "@formant/ui-sdk";

interface IIconButtonProps {
  iconName: any;
  onClick: () => void;
  id?: string;
  className?: string;
}

export const IconButton: FC<IIconButtonProps> = ({
  iconName,
  id,
  className,
  onClick,
}) => {
  return (
    <div className={`${styles["icon-button"]} ${className}`} id={id} onClick={onClick}>
      <Icon name={iconName} />
    </div>
  );
};
