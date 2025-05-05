import {
  Dialog,
  DialogTitle,
  DialogContent,
  Stack,
  DialogContentText,
  DialogActions,
  Button,
} from "@formant/ui-sdk";
import { FC } from "react";

interface IProps {
  title: string;
  description: string;
  openDialog: boolean;
  handleCloseDialog: () => void;
  onOk: () => void;
}

export const DialogComponent: FC<IProps> = ({
  openDialog,
  handleCloseDialog,
  description,
  onOk,
  title,
}) => {
  return (
    <Dialog open={openDialog} onClose={handleCloseDialog} fullWidth={true}>
      <DialogTitle
        sx={{
          color: "#bac4e2",
          fontSize: 13,
        }}
      >
        {title}
      </DialogTitle>
      <DialogContent>
        <Stack spacing={2}>
          <DialogContentText>{description}</DialogContentText>
        </Stack>
      </DialogContent>
      <DialogActions>
        <Stack direction="row" spacing={2}>
          <Button size="medium" variant="contained" onClick={handleCloseDialog}>
            Cancel
          </Button>
          <Button
            size="medium"
            variant="contained"
            color="error"
            onClick={onOk}
            sx={{
              color: "black",
              ":hover": {
                boxShadow: " 0 0 0 0.4rem #ea719d",
                backgroundColor: "#ea719d",
              },
            }}
          >
            ok
          </Button>
        </Stack>
      </DialogActions>
    </Dialog>
  );
};
