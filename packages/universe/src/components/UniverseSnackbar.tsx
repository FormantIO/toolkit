import { Snackbar } from "@formant/ui-sdk";
import * as React from "react";
import { useRecoilState } from "recoil";
import { snackbarAtom } from "../state/snackbar";

export function UniverseSnackbar() {
  const [snackbar, setSnackbar] = useRecoilState(snackbarAtom);
  const onClose = () => {
    setSnackbar({
      open: false,
    });
  };
  return (
    <Snackbar
      anchorOrigin={{ vertical: "top", horizontal: "center" }}
      open={snackbar.open}
      message={snackbar.message}
      autoHideDuration={6000}
      onClose={onClose}
    />
  );
}
