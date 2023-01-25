import { Authentication } from "@formant/data-sdk";

export const authenticate = async (_callback: any) => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      return _callback();
    }
  } catch (e) {
    throw e;
  }
};
