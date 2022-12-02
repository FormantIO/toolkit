import { Icon } from "../../main";
import styles from "./IconInput.module.scss";
import React from "react"

export const IconInput = (props: any) => {
  const { onChange, onClick, value } = props;

  return (
    <div className={styles["icon-input"]}>
      <Icon name="search" />
      <input value={value} onChange={onChange} {...props} />
      <button onClick={onClick}>
        {value !== null && value !== "" && <Icon name="close" />}
      </button>
    </div>
  );
};
