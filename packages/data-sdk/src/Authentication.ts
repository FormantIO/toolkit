import { FORMANT_API_URL } from "./config";

export interface User {
  firstName: string;
  lastName: string;
  email: string;
  organizationId: string;
  id: string;
}

export class Authentication {
  static token: string | undefined;
  static currentUser: User | undefined;
  static defaultDeviceId: string | undefined;
  static waitingForAuth: ((result: boolean) => void)[] = [];

  public static async login(email: string, password: string) {
    try {
      const result = await fetch(`${FORMANT_API_URL}/v1/admin/auth/login`, {
        method: "POST",
        body: JSON.stringify({ email, password }),
        headers: {
          "Content-Type": "application/json",
        },
      });
      const auth = await result.json();
      await Authentication.loginWithToken(
        auth.authentication.accessToken as string
      );
    } catch (e: any) {
      console.error(e);
      Authentication.waitingForAuth.forEach((_) => _(false));
      Authentication.waitingForAuth = [];
    }
  }

  public static async loginWithToken(token: string) {
    const tokenData = JSON.parse(atob(token.split(".")[1]));
    try {
      const data = await fetch(
        `${FORMANT_API_URL}/v1/admin/users/${tokenData.sub}`,
        {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
            Authorization: "Bearer " + token,
          },
        }
      );
      Authentication.currentUser = await data.json();
      Authentication.token = token;
      Authentication.waitingForAuth.forEach((_) => _(true));
    } catch (e: any) {
      console.error(e);
      Authentication.waitingForAuth.forEach((_) => _(false));
    }
    Authentication.waitingForAuth = [];
  }

  static isAuthenticated(): boolean {
    return Authentication.token !== undefined;
  }

  static getCurrentUser(): User | undefined {
    return Authentication.currentUser;
  }

  static async waitTilAuthenticated(): Promise<boolean> {
    if (Authentication.token !== undefined) {
      return true;
    } else {
      return new Promise((resolve) => {
        Authentication.waitingForAuth.push(function (result: boolean) {
          resolve(result);
        });
      });
    }
  }
}
