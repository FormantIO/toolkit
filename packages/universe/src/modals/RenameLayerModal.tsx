import { DialogContentText, Stack, TextField } from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import { Modal } from "../modals/Modal";

interface IRenameLayerModalProps {
  name: string;
  onRenameLayer: (name: string) => void;
  onCancel: () => void;
}
interface IRenameLayerModalState {
  currentName: string;
}

export class RenameLayerModal extends Component<
  IRenameLayerModalProps,
  IRenameLayerModalState
> {
  constructor(props: IRenameLayerModalProps) {
    super(props);
    this.state = {
      currentName: props.name,
    };
  }

  private onRenameClick = () => {
    this.props.onRenameLayer(this.state.currentName);
  };

  private onChangeName = (ev: React.ChangeEvent<HTMLInputElement>) => {
    this.setState({
      currentName: ev.target.value,
    });
  };

  public render() {
    const { currentName } = this.state;
    const { onCancel } = this.props;
    return (
      <Modal
        open
        title="Rename Layer"
        acceptText="Rename"
        onAccept={this.onRenameClick}
        onClose={onCancel}
      >
        <Stack spacing={2}>
          <DialogContentText>
            Rename an layer in your universe
          </DialogContentText>
          <div>
            <TextField value={currentName} onChange={this.onChangeName} />
          </div>
        </Stack>
      </Modal>
    );
  }
}
