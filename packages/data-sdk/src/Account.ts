import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { IAccount } from "./model/IAccount";
import { IAccountTree } from "./model/IAccountTree";

export class Account {
  static async listAccounts(): Promise<IAccount[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/accounts`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const accounts = await data.json();
    return accounts.items;
  }

  static async createAccounts(account: IAccount): Promise<IAccount> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/accounts`, {
      method: "POST",
      body: JSON.stringify(account),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async getAccount(id: string): Promise<IAccount> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/accounts/${id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async patchAccount(
    id: string,
    account: Partial<Account>
  ): Promise<IAccount> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/accounts/${id}`, {
      method: "PATCH",
      body: JSON.stringify(account),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }
  static async deleteAccount(id: string): Promise<void> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const result = await fetch(`${FORMANT_API_URL}/v1/admin/accounts/${id}`, {
      method: "DELETE",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    if (!result.ok) {
      throw new Error("Unable to delete account");
    }
  }

  static async getAccountTree(id: string): Promise<IAccountTree> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(
      `${FORMANT_API_URL}/v1/admin/accounts/${id}/tree`,
      {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      }
    );
    return await data.json();
  }
}
