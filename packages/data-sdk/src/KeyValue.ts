import { Authentication } from "./Authentication";
import { DataSdk } from "./DataSdk";
import { defined } from "../../common/defined";
import { ITags } from "./model/ITags";

export class KeyValue {
  public static async set(key: string, value: string, tags?: ITags) {
    const result = await fetch(`${DataSdk.adminApi}/key-value`, {
      method: "POST",
      body: JSON.stringify({
        organizationId: defined(Authentication.currentUser).organizationId,
        key,
        value,
        tags,
      }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const keyValue = await result.json();
    if (result.status !== 200) {
      throw new Error(keyValue.message);
    }
  }

  public static async get(key: string): Promise<string> {
    const result = await fetch(`${DataSdk.adminApi}/key-value/${key}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const keyValue = await result.json();
    if (result.status !== 200) {
      throw new Error(keyValue.message);
    }
    return keyValue.value;
  }

  public static async list(): Promise<string[]> {
    const result = await fetch(`${DataSdk.adminApi}/key-value`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const keyValueList = await result.json();
    if (result.status !== 200) {
      throw new Error(keyValueList.message);
    }
    return keyValueList.items;
  }

  public static async delete(key: string) {
    const result = await fetch(`${DataSdk.adminApi}/key-value/${key}`, {
      method: "DELETE",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    if (!result.ok) {
      throw new Error("Unable to handle request");
    }
    return;
  }

  public static async query(keys: string[]) {
    const result = await fetch(`${DataSdk.adminApi}/key-value/query`, {
      method: "POST",
      body: JSON.stringify({ keys }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    if (!result.ok) {
      throw new Error("Unable to handle request");
    }
    const jsonResponse = await result.json();
    return jsonResponse.items;
  }
}
