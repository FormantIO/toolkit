import { createSlice } from "@reduxjs/toolkit";

const initialState = {
  modalState: {
    title: "",
    content: "",
    streamName: "",
    open: false,
    paramType: null,
  },
};

export const modalSlice = createSlice({
  name: "modal",
  initialState,
  reducers: {
    handleOpenModal: (state) => {
      state.modalState.open = true;
    },
    handleCloseModal: (state) => {
      state.modalState.open = false;
    },
    setModalState: (state, action) => {
      state.modalState = action.payload.state;
    },
  },
});

export const { handleCloseModal, handleOpenModal, setModalState } =
  modalSlice.actions;

export default modalSlice.reducer;
