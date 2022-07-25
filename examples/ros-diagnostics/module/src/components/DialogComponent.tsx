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
  openDialog: boolean;
  handleCloseDialog: () => void;
  topicName: string;
  deleteTopic: () => void;
}

export const DialogComponent: FC<IProps> = ({
  openDialog,
  handleCloseDialog,
  topicName,
  deleteTopic,
}) => {
  return (
    <Dialog open={openDialog} onClose={handleCloseDialog} fullWidth={true}>
      <DialogTitle
        sx={{
          color: "#bac4e2",
          fontSize: 13,
        }}
      >
        DELETE TOPIC
      </DialogTitle>
      <DialogContent>
        <Stack spacing={2}>
          <DialogContentText>
            {`Delete topic "${topicName}"`}?
          </DialogContentText>
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
            color="secondary"
            onClick={deleteTopic}
          >
            ok
          </Button>
        </Stack>
      </DialogActions>
    </Dialog>
  );
};
