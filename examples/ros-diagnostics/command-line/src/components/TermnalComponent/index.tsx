import { Authentication, Device, Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";
import styles from "./index.module.scss";

export const TerminalComponent = () => {
  const [user, setUser] = useState("");
  const [host, setHost] = useState("");
  const [consoleState, setConsoleState] = useState(false);

  useEffect(() => {
    getuser();
  }, []);

  const getuser = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const device = await Fleet.getCurrentDevice();
      setUser(Authentication.currentUser!.firstName);
      setHost(device.name);
    }
  };
  return (
    <div
      onFocus={() => setConsoleState(true)}
      onBlur={() => setConsoleState(false)}
      className={`${styles.terminal} ${
        styles[consoleState ? "terminal-focused" : 0]
      } `}
    >
      <span>{`${user}@${host}:~$`}</span>
      <textarea onSubmit={() => console.log("hello")} name="command" />
    </div>
  );
};
