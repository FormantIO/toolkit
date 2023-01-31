import { FC } from "react";
import { Narrow, Wide, Hidden } from "../icons";
import styles from "./index.module.scss";

type Active = "hidden" | "narrow" | "wide";

interface IThreeWaySwitch {
  active: Active;
  handleOnHide: () => void;
  handleOnNarrow: () => void;
  handleOnWide: () => void;
}

export const ThreeWaySwitch: FC<IThreeWaySwitch> = ({
  active,
  handleOnHide,
  handleOnNarrow,
  handleOnWide,
}) => {
  return (
    <div className={styles.switch}>
      <Hidden handleOnHide={handleOnHide} active={active === "hidden"} />
      <Narrow handleOnNarrow={handleOnNarrow} active={active === "narrow"} />
      <Wide handleOnWide={handleOnWide} active={active === "wide"} />
    </div>
  );
};
