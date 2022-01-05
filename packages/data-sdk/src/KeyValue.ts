import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { defined } from "../../common/defined";

export class KeyValue {
  public static async set(key: string, value: string) {
    try {
      const result = await fetch(FORMANT_API_URL + "/v1/admin/key-value", {
        method: "POST",
        body: JSON.stringify({
          organizationId: defined(Authentication.currentUser).organizationId,
          key,
          value,
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
    } catch (e: any) {
      throw e;
    }
  }

  public static async get(key: string): Promise<string> {
    try {
      const result = await fetch(
        FORMANT_API_URL + `/v1/admin/key-value/${key}`,
        {
          method: "GET",
          body: JSON.stringify({
            organizationId: defined(Authentication.currentUser).organizationId,
          }),
          headers: {
            "Content-Type": "application/json",
            Authorization: "Bearer " + Authentication.token,
          },
        }
      );
      const keyValue = await result.json();
      if (result.status !== 200) {
        throw new Error(keyValue.message);
      }
      return keyValue.value;
    } catch (e: any) {
      throw e;
    }
  }
}
