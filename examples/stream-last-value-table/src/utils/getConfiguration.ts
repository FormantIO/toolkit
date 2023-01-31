import { Authentication, KeyValue } from "@formant/data-sdk";

export const getConfiguration = async (confiurationKey: string) => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      return JSON.parse(await KeyValue.get(confiurationKey));
    }
    return [];
  } catch (e) {
    return [];
  }
};
