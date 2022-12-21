import { configureStore } from "@reduxjs/toolkit";
import moduleNameReducer from "../features/moduleName/moduleNameSlice";
import configurationReducer from "../features/configuration/configurationSlice";
import modalReducer from "../features/modal/modalSlice";

export const store = configureStore({
  reducer: {
    moduleName: moduleNameReducer,
    configuration: configurationReducer,
    modal: modalReducer,
  },
});

export type AppState = ReturnType<typeof store.getState>;
