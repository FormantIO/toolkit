import { FORMANT_API_URL } from "./config";
import { App } from "./App";

export interface User {
  firstName: string;
  lastName: string;
  email: string;
  organizationId: string;
  id: string;
}

export interface IAuthentication {
  accessToken: string;
  organizationId: string;
  refreshToken: string;
  userId: string;
}

export class Authentication {
  static token: string | undefined;
  static refreshToken: string | undefined;
  static currentUser: User | undefined;
  static currentOrganization: string | undefined;
  static isShareToken: boolean = false;
  static defaultDeviceId: string | undefined;
  static waitingForAuth: ((result: boolean) => void)[] = [];

  public static async login(
    email: string,
    password: string
  ): Promise<IAuthentication | Error> {
    try {
      const result = await fetch(`${FORMANT_API_URL}/v1/admin/auth/login`, {
        method: "POST",
        body: JSON.stringify({ email, password }),
        headers: {
          "Content-Type": "application/json",
        },
      });
      const auth = await result.json();
      if (result.status !== 200) {
        throw new Error(auth.message);
      }
      await Authentication.loginWithToken(
        auth.authentication.accessToken as string,
        auth.authentication.refreshToken as string
      );

      return auth.authentication;
    } catch (e: any) {
      Authentication.waitingForAuth.forEach((_) => _(false));
      Authentication.waitingForAuth = [];

      return Promise.reject(e);
    }
  }

  public static async loginWithToken(token: string, refreshToken?: string) {
    const tokenData = JSON.parse(atob(token.split(".")[1]));
    try {
      let userId: string | undefined;
      Authentication.isShareToken =
        tokenData["formant:claims"] &&
        tokenData["formant:claims"].type == "share";
      if (tokenData["formant:claims"]) {
        Authentication.currentOrganization =
          tokenData["formant:claims"].organizationId;
      }
      if (tokenData["custom:organization_id"]) {
        Authentication.currentOrganization =
          tokenData["custom:organization_id"];
      }
      if (!Authentication.isShareToken) {
        userId = tokenData.sub;
      }
      if (tokenData["formant:claims"] && tokenData["formant:claims"].userId) {
        userId = tokenData["formant:claims"].userId;
      }
      if (userId) {
        const result = await fetch(
          `${FORMANT_API_URL}/v1/admin/users/${userId}`,
          {
            method: "GET",
            headers: {
              "Content-Type": "application/json",
              Authorization: "Bearer " + token,
            },
          }
        );
        const data = await result.json();
        if (result.status !== 200) {
          throw new Error(data.message);
        }
        Authentication.currentUser = data;
      }
      Authentication.token = token;
      Authentication.waitingForAuth.forEach((_) => _(true));
    } catch (e: any) {
      console.error(e);
      Authentication.waitingForAuth.forEach((_) => _(false));
    }
    Authentication.waitingForAuth = [];

    if (refreshToken) {
      Authentication.refreshToken = refreshToken;
      setInterval(async () => {
        if (Authentication.refreshToken) {
          const result = await fetch(
            `${FORMANT_API_URL}/v1/admin/auth/refresh`,
            {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                refreshToken: Authentication.refreshToken,
              }),
            }
          );
          const refreshData = await result.json();
          Authentication.token = refreshData.authentication.accessToken;
        }
      }, 1000 * 60 * 60);
    }
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

  static async listenForRefresh() {
    // refresh token every hour
    App.addAccessTokenRefreshListener((token: string) => {
      this.loginWithToken(token);
    });
    setInterval(async () => {
      App.refreshAuthToken();
    }, 1000 * 60 * 60);
  }
}
