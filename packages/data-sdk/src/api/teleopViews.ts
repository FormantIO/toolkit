// packages/data-sdk/src/api/queryViews.ts

import { Authentication } from "../Authentication";
import { FORMANT_API_URL } from "../config";
import { RtcStreamType } from "../connector/model/IUniverseData";
import { IsoDate } from "../model/IsoDate";
import { ITags } from "../model/ITags";

export interface IDictionary<T = string> {
  [key: string]: T;
}

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

// Define the ModuleConfigItem as a union
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

// Define a type for the configuration values
export type ConfigValue = string | number | boolean | any[];

// Define the ModuleConfig type
export type ModuleConfig = {
  [key: string]: ConfigValue;
};

export async function request<T>(
  endpoint: string,
  options?: RequestInit
): Promise<T> {
  if (!Authentication.token) {
    throw new Error("Not authenticated");
  }

  const data = await fetch(`${FORMANT_API_URL}/v1${endpoint}`, {
    ...options,
    headers: {
      "Content-Type": "application/json",
      Authorization: `Bearer ${Authentication.token}`,
      ...options?.headers,
    },
  });

  if (!data.ok) {
    throw new Error(`Error: ${data.statusText}`);
  }

  if (data.status === 204 || data.headers.get("content-length") === "0") {
    return null as T;
  }

  return (await data.json()) as T;
}

export async function fetchTeleopViews(): Promise<ITeleopView[]> {
  const data = await request<ITeleopViewResponse>("/admin/teleop-views");
  return data.items;
}

export async function getTeleopView(id: string): Promise<ITeleopView> {
  return await request<ITeleopView>(`/admin/teleop-views/${id}`);
}

export async function createTeleopView(
  view: Omit<ITeleopView, "id" | "organizationId" | "createdAt" | "updatedAt">
): Promise<ITeleopView> {
  return await request<ITeleopView>("/admin/teleop-views", {
    method: "POST",
    body: JSON.stringify(view),
  });
}

export async function updateTeleopView(
  id: string,
  view: Partial<ITeleopView>
): Promise<ITeleopView> {
  return await request<ITeleopView>(`/admin/teleop-views/${id}`, {
    method: "PATCH",
    body: JSON.stringify(view),
  });
}

export async function deleteTeleopView(id: string): Promise<void> {
  await request(`/admin/teleop-views/${id}`, {
    method: "DELETE",
  });
}
