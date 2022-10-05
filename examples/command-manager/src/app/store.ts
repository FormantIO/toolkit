import { configureStore } from "@reduxjs/toolkit";
import moduleNameReducer from "../features/moduleName/moduleNameSlice";
import configurationReducer from "../features/configuration/configurationSlice";

export const store = configureStore({
  reducer: {
    moduleName: moduleNameReducer,
    configuration: configurationReducer,
  },
});


