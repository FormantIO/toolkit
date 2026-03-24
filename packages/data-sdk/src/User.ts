import { Authentication } from "./Authentication";
import { DataSdk } from "./DataSdk";
import { IUser } from "./model/IUser";

export class User {
  static async listUsers(): Promise<IUser[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${DataSdk.adminApi}/users`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const users = await data.json();
    return users.items;
  }

  static async createUser(user: IUser): Promise<IUser> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${DataSdk.adminApi}/users`, {
      method: "POST",
      body: JSON.stringify(user),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async getUser(id: string): Promise<IUser> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${DataSdk.adminApi}/users/${id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async patchUser(id: string, user: Partial<IUser>): Promise<IUser> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${DataSdk.adminApi}/users/${id}`, {
      method: "PATCH",
      body: JSON.stringify(user),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async deleteUser(id: string): Promise<void> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    await fetch(`${DataSdk.adminApi}/users/${id}`, {
      method: "PATCH",
      body: JSON.stringify({ enabled: false, roleId: null, teamId: null }),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
  }
}
