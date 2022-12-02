import { Icon } from "@formant/ui-sdk";
import styles from "./IconInput.module.scss";

export const IconInput = (props: any) => {
  const { onChange, onClear, value } = props;

  return (
    <div className={styles["icon-input"]}>
      <Icon name="search" />
      <input value={value} onChange={onChange} {...props} />
      <button onClick={onClear}>
        {value !== null && value !== "" && <Icon name="close" />}
      </button>
    </div>
  );
};
