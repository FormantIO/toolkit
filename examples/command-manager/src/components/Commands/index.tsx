import { TextField, Select } from "@formant/ui-sdk";
import { FC, useCallback, useState } from "react";
import styles from "./index.module.scss";
import { CommandIssuer } from "./CommandIssuer";
import { useSelector } from "react-redux";
import { AppState } from "../../app/store";
import { IConfiguration } from "../../types";
import { Modal } from "../Modal";
import { useDispatch } from "react-redux";
import { handleCloseModal } from "../../features/modal/modalSlice";
import { Device } from "@formant/data-sdk";
import { useDropDownItems } from "../../hooks/useDropDownItem";

interface ICommandsProps {
  configuration: IConfiguration;
  device: Device;
}

export const Commands: FC<ICommandsProps> = ({ configuration, device }) => {
  const dispatch = useDispatch();
  const modalState = useSelector((state: AppState) => state.modal.modalState);
  const [param, setParam] = useState("");
  const dropDownItems = useDropDownItems(device);
  const handleClose = useCallback(() => {
    dispatch(handleCloseModal());
  }, [modalState]);

  const handleIssueCommand = useCallback(() => {
    if (!device) return;

    device.sendCommand(modalState.title, param);

    setParam("");
    handleClose();
  }, [device, modalState.title, param]);

  const handleIssueCommands = useCallback(
    (title: string, p: string) => {
      if (!device) return;

      device.sendCommand(title, p);

      setParam("");
      handleClose();
    },
    [device, modalState.title, param]
  );

  return (
    <div className={styles.commands}>
      <Modal
        onOk={handleIssueCommand}
        title={modalState.title}
        onCancel={handleClose}
        open={modalState.open}
        content={modalState.content}
      >
        {modalState.paramType === "textField" ? (
          <TextField
            variant="filled"
            autoFocus
            id="Parameter"
            label="Parameter"
            fullWidth
            onChange={(e) => setParam(e.target.value)}
          />
        ) : modalState.paramType === "dropDown" ? (
          <Select
            onChange={(value) => setParam(value as string)}
            items={dropDownItems}
            label="Parameter"
            fullWidth
          />
        ) : (
          <></>
        )}
      </Modal>
      {configuration.commands.map((_) => (
        <CommandIssuer onClick={handleIssueCommands} key={_.name} command={_} />
      ))}
    </div>
  );
};
