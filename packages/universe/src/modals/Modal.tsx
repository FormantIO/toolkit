import * as React from "react";
import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Stack,
} from "@formant/ui-sdk";

export interface IModalProps {
  title: string;
  open: boolean;
  children: React.ReactNode;
  onClose: () => void;
  onAccept: () => void;
  acceptText: string;
  acceptDisabled?: boolean;
}

export function Modal(props: IModalProps) {
  const { onClose, children, open, title, onAccept, acceptText } = props;
  return (
    <Dialog open={open} onClose={onClose}>
      <DialogTitle>{title}</DialogTitle>
      <DialogContent>{children}</DialogContent>
      <DialogActions>
        <Stack direction="row" spacing={2}>
          <Button size="large" variant="contained" onClick={onClose}>
            Cancel
          </Button>
          <Button
            color="secondary"
            size="large"
            variant="contained"
            onClick={onAccept}
            disabled={props.acceptDisabled || false}
          >
            {acceptText}
          </Button>
        </Stack>
      </DialogActions>
    </Dialog>
  );
}
