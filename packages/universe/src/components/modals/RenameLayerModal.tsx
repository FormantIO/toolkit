import { DialogContentText, Stack, TextField } from "@formant/ui-sdk";
import * as React from "react";
import { Modal } from "./Modal";

interface IRenameLayerModalProps {
  name: string;
  onRenameLayer: (name: string) => void;
  onCancel: () => void;
}

export function RenameLayerModal(props: IRenameLayerModalProps) {
  const [currentName, setCurrentName] = React.useState(props.name);

  const onRenameClick = () => {
    props.onRenameLayer(currentName);
  };

  const onChangeName = (ev: React.ChangeEvent<HTMLInputElement>) => {
    setCurrentName(ev.target.value);
  };

  const { onCancel } = props;
  return (
    <Modal
      open
      title="Rename Layer"
      acceptText="Rename"
      onAccept={onRenameClick}
      onClose={onCancel}
    >
      <Stack spacing={2}>
        <DialogContentText>Rename an layer in your universe</DialogContentText>
        <div>
          <TextField value={currentName} onChange={onChangeName} />
        </div>
      </Stack>
    </Modal>
  );
}
