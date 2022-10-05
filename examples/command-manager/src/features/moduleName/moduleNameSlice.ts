import { createSlice } from "@reduxjs/toolkit";
import { App } from "@formant/data-sdk";

const initialState = {
  moduleName: App.getCurrentModuleContext(),
};

export const moduleNameSlice = createSlice({
  name: "moduleName",
  initialState,
  reducers: {
    setModuleName: (state, action) => {
      state.moduleName = action.payload.moduleName;
    },
  },
});

export const { setModuleName } = moduleNameSlice.actions;

export default moduleNameSlice.reducer;
