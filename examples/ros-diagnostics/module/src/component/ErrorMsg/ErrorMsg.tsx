import styles from "./ErrorMsg.module.scss";
import { Component, ReactNode } from "react";
import { WarningIcon } from "./WarningIcon";

interface IErrorMsgProps {
  msg: string;
}

export class ErrorMsg extends Component<IErrorMsgProps> {
  render(): ReactNode {
    return (
      <div className={styles.container}>
        <div
          className={`${styles.error} ${
            styles[
              this.props.msg.includes("Error") ? "error-found" : "error-loading"
            ]
          } `}
        >
          <div
            className={`${styles["error-icon"]} ${
              styles[
                this.props.msg.includes("Error")
                  ? "error-icon-show"
                  : "error-icon-hide"
              ]
            }`}
          >
            <WarningIcon />
          </div>
          <p>{this.props.msg}</p>
        </div>
      </div>
    );
  }
}
