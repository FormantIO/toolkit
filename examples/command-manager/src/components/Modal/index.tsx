import {
  Dialog,
  DialogTitle,
  DialogContent,
  Stack,
  DialogContentText,
  TextField,
  DialogActions,
  Button,
} from "@formant/ui-sdk";
import { FC } from "react";

interface IModalProps {
  open: boolean;
  title: string;
  content: string;
  onCancel: () => void;
  onOk: () => void;
  children: JSX.Element;
}

export const Modal: FC<IModalProps> = ({
  open,
  title,
  content,
  onCancel,
  onOk,
  children,
}) => {
  return (
    <Dialog open={open} onClose={onCancel}>
      <DialogTitle>{title}</DialogTitle>
      <DialogContent>
        <Stack spacing={2}>
          <DialogContentText>{content}</DialogContentText>
          {children}
        </Stack>
      </DialogContent>
      <DialogActions>
        <Stack direction="row" spacing={2}>
          <Button size="large" variant="contained" onClick={onCancel}>
            Cancel
          </Button>
          <Button
            size="large"
            variant="contained"
            color="secondary"
            onClick={onOk}
          >
            OK
          </Button>
        </Stack>
      </DialogActions>
    </Dialog>
  );
};
