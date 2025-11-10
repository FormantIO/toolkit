import styles from "./ErrorMsg.module.scss";
import { Component, ReactNode } from "react";
import { WarningIcon } from "./WarningIcon";
import { Icon } from "@formant/ui-sdk";

interface IErrorMsgProps {
  msg: string;
}

export class ErrorMsg extends Component<IErrorMsgProps> {
  render(): ReactNode {
    return (
      <div className={styles.container}>
        <p style={{ fontWeight: 600, fontSize: 18 }}>Offline</p>
      </div>
    );
  }
}
