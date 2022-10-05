import { createSlice } from "@reduxjs/toolkit";
import { activeCommands } from "../../types";

const initialState: activeCommands = {
  activeCommands: [],
};

export const configurationSlice = createSlice({
  name: "configuration",
  initialState,
  reducers: {
    updateActiveCommands: (state, action) => {
      state.activeCommands = action.payload.items;
    },
    handleCommandChange: (state, action) => {
      const { id, checked } = action.payload;
      switch (checked) {
        case true:
          state.activeCommands.push(id);
          return;
        case false:
          state.activeCommands = state.activeCommands.filter((_) => _ !== id);
          return;
        default:
          return state;
      }
    },
  },
});

export const { updateActiveCommands, handleCommandChange } =
  configurationSlice.actions;

export default configurationSlice.reducer;
