import { Authentication } from "../Authentication";
import { DataSdk } from "../DataSdk";
import { RtcStreamType } from "../connector/model/IUniverseData";
import { IsoDate } from "../model/IsoDate";
import { ITags } from "../model/ITags";

export interface ITeleopViewModule {
  id: string;
  type: string;
  position: { x: number; y: number };
  size: { width: number; height: number };
  isButton?: boolean;
  label?: string;
  isTextInput?: boolean;
  config: ModuleConfig;
}

export interface ITeleopViewConfiguration {
  modules: ITeleopViewModule[];
  modulePickerPosition: { x: number; y: number };
}

export interface ITeleopView {
  id: string;
  organizationId: string;
  name: string;
  configuration: ITeleopViewConfiguration;
  createdAt?: IsoDate;
  updatedAt?: IsoDate;
  tags?: ITags;
}

export interface ITeleopViewResponse {
  items: ITeleopView[];
}

export type ModuleConfigItem =
  | {
      id: string;
      name: string;
      type: "realtime-streams";
      multiple?: boolean;
      supportedTypes: RtcStreamType[];
    }
  | { id: string; name: string; type: "named-string"; defaultValue: string }
  | { id: string; name: string; type: "named-boolean"; defaultValue: boolean }
  | { id: string; name: string; type: "named-number"; defaultValue: number };

export type ConfigValue = string | number | boolean | unknown[];

export type ModuleConfig = {
  [key: string]: ConfigValue;
};

export async function fetchTeleopViews(): Promise<ITeleopView[]> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const data = await fetch(`${DataSdk.adminApi}/teleop-views`, {
    headers: {
      "Content-Type": "application/json",
      Authorization: `Bearer ${Authentication.token}`,
    },
  });

  if (!data.ok) {
    throw new Error(`Error: ${data.statusText}`);
  }

  const response = (await data.json()) as ITeleopViewResponse;
  return response.items;
}
