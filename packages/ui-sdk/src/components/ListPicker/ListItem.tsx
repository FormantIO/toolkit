import styles from "./ListItem.module.scss";
import { Switch, Typography } from "../../main";
import React, { FC } from "react";

interface IListItemProps {
  name: string;
  enabled: boolean;
  onChange: (
    event: React.ChangeEvent<HTMLInputElement>,
    checked: boolean
  ) => void;
}

export const ListItem: FC<IListItemProps> = ({
  name,
  enabled,
  onChange,
}) => {
  return (
    <div className={styles["list-item"]}>
      <Typography variant="body2">{name}</Typography>{" "}
      <Switch
        size="small"
        onChange={onChange}
        checked={enabled}
        value={enabled}
      />
    </div>
  );
};
