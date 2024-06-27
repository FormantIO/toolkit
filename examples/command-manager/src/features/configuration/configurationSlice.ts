import { createSlice } from "@reduxjs/toolkit";
import { IConfiguration } from "../../types";

const initialState: any = {
  commands: [],
};

export const configurationSlice = createSlice({
  name: "configuration",
  initialState,
  reducers: {
    setCommands: (state, action) => {
      state.commands = action.payload.items;
    },
    handleCommandChange: (state, action) => {
      const { id, checked } = action.payload;
      switch (checked) {
        case true:
          state.commands.push(id);
          return;
        case false:
          state.commands = state.commands.filter((_: {id: string}) => _ !== id);
          return;
        default:
          return state;
      }
    },
  },
});

export const { setCommands, handleCommandChange } = configurationSlice.actions;

export default configurationSlice.reducer;
