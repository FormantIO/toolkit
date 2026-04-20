import { Authentication, KeyValue } from "@formant/data-sdk";
import * as React from "react";

export const useFormantStorage = () => {
  const [keyValueStore, setKeyValueStore] = React.useState({
    currentList: [] as string[],
    list: () => {},
    set: (key: string, value: string) => {},
    get: (key: string) => {},
    delete: (key: string) => {},
  });

  React.useEffect(() => {
    getAuth();
  }, [keyValueStore]);

  const getAuth = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const _list = await list();
      setKeyValueStore({
        currentList: _list,
        list,
        set,
        get,
        delete: del,
      });
    }
  };
  const list = async () => {
    const list = await KeyValue.list();
    return list;
  };
  const set = async (key: string, value: string) => {
    return await KeyValue.set(key, value);
  };

  const get = async (key: string) => {
    return await KeyValue.get(key);
  };

  const del = async (key: string) => {
    return await KeyValue.delete(key);
  };
  return keyValueStore;
};
