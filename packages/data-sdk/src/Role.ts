import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { IRole } from "./model/IRole";

export class Role {
  static async listRoles(): Promise<IRole[]> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/roles`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    const roles = await data.json();
    return roles.items;
  }

  static async createRole(role: IRole): Promise<IRole> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/roles`, {
      method: "POST",
      body: JSON.stringify(role),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async getRole(id: string): Promise<IRole> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/roles/${id}`, {
      method: "GET",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async patchRole(id: string, role: Partial<IRole>): Promise<IRole> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    const data = await fetch(`${FORMANT_API_URL}/v1/admin/roles/${id}`, {
      method: "PATCH",
      body: JSON.stringify(role),
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
    return await data.json();
  }

  static async deleteRole(id: string): Promise<void> {
    if (!Authentication.token) {
      throw new Error("Not authenticated");
    }
    await fetch(`${FORMANT_API_URL}/v1/admin/roles/${id}`, {
      method: "DELETE",
      headers: {
        "Content-Type": "application/json",
        Authorization: "Bearer " + Authentication.token,
      },
    });
  }
}
