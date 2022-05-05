import { atom } from "recoil";

interface SnackbarState {
  message?: string;
  open: boolean;
}

export const snackbarAtom = atom<SnackbarState>({
  key: "snackbarMessage",
  default: { open: false },
});
