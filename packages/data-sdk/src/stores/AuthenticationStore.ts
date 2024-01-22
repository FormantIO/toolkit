import { decode } from "base-64";

import { IAuthenticationStore } from "./IAuthenticationStore";
import { AuthenticationResult } from "./AuthenticationResult";
import {
  LoginFailureError,
  LoginChallengedError,
} from "./AuthenticationErrors";

import { IUser } from "../model/IUser";
import { IAuthentication } from "./IAuthentication";
import { IConfirmForgotPasswordRequest } from "./IConfirmForgotPasswordRequest";
import { IRespondToNewPasswordRequiredChallengeRequest } from "./IRespondToNewPasswordRequiredChallengeRequest";
import { ICheckSsoResult } from "./ICheckSsoResult";

interface IAuthenticationStoreOptions {
  apiUrl: string;

  refreshAuthToken: () => void;
  addAccessTokenRefreshListener: (
    callback: (token: string) => void
  ) => () => void;
}

export class AuthenticationStore implements IAuthenticationStore {
  private _refreshToken: string | undefined;
  private _isShareToken: boolean = false;
  private _currentOrganization: string | undefined;
  private _currentUser: IUser | undefined;
  private _defaultDeviceId: string | undefined;
  private _token: string | undefined;
  private _waitingForAuth: Set<(result: boolean) => void> = new Set();
  private _refreshTimer: ReturnType<typeof setTimeout> | undefined;

  private readonly _apiUrl: string;
  private readonly _refreshAuthToken: () => void;
  private readonly _addAccessTokenRefreshListener: (
    callback: (token: string) => void
  ) => () => void;

  constructor({
    apiUrl,
    refreshAuthToken,
    addAccessTokenRefreshListener,
  }: IAuthenticationStoreOptions) {
    this._apiUrl = apiUrl;
    this._refreshAuthToken = refreshAuthToken;
    this._addAccessTokenRefreshListener = addAccessTokenRefreshListener;
  }

  get token(): string | undefined {
    return this._token;
  }

  get currentUser(): IUser | undefined {
    return this._currentUser;
  }

  get currentOrganization(): string | undefined {
    return this._currentOrganization;
  }

  get defaultDeviceId(): string | undefined {
    return this._defaultDeviceId;
  }

  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  get refreshToken(): string | undefined {
    return this._refreshToken;
  }

  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  get isShareToken(): boolean {
    return this._isShareToken;
  }

  login(email: string, password: string): Promise<IAuthentication>;
  login(
    email: string,
    password: string,
    options: { advanced: true }
  ): Promise<AuthenticationResult>;
  async login(
    email: string,
    password: string,
    options: { advanced?: boolean } = {}
  ): Promise<IAuthentication | AuthenticationResult> {
    const { advanced = false } = options;

    try {
      const result = await fetch(`${this._apiUrl}/v1/admin/auth/login`, {
        method: "POST",
        body: JSON.stringify({ email, password }),
        headers: {
          "Content-Type": "application/json",
        },
      });

      const auth = await result.json();
      if (result.status !== 200) {
        throw new LoginFailureError(auth.message);
      }

      if ("challenge" in auth) {
        throw new LoginChallengedError(auth.challenge);
      }

      const { authentication } = auth;
      await this.loginWithToken(
        authentication.accessToken as string,
        authentication.refreshToken as string
      );

      return !advanced
        ? authentication
        : {
            result: "success",
            authentication,
          };
    } catch (err: unknown) {
      if (!advanced) {
        console.error("login() failed", { err });
      }

      this._waitingForAuth.forEach((_) => _(false));
      this._waitingForAuth.clear();

      if (!advanced) {
        throw err;
      }

      if (err instanceof LoginChallengedError) {
        return {
          result: "challenged",
          challenge: err.challenge,
        };
      }

      return {
        result: "failure",
        reason:
          err instanceof LoginFailureError
            ? err.reason
            : err instanceof Error
            ? err.message
            : String(err),
      };
    }
  }

  async loginWithToken(token: string, refreshToken?: string) {
    const tokenData = JSON.parse(decode(token.split(".")[1]));
    try {
      let userId: string | undefined;
      this._isShareToken =
        tokenData["formant:claims"] &&
        tokenData["formant:claims"].type == "share";

      if (tokenData["formant:claims"]) {
        this._currentOrganization = tokenData["formant:claims"].organizationId;
      }
      if (tokenData["custom:organization_id"]) {
        this._currentOrganization = tokenData["custom:organization_id"];
      }

      if (!this._isShareToken) {
        userId = tokenData.sub;
      }
      if (tokenData["formant:claims"] && tokenData["formant:claims"].userId) {
        userId = tokenData["formant:claims"].userId;
      }

      if (userId && this._currentUser?.id !== userId) {
        const result = await fetch(`${this._apiUrl}/v1/admin/users/${userId}`, {
          method: "GET",
          headers: {
            "Content-Type": "application/json",
            Authorization: "Bearer " + token,
          },
        });
        const data = await result.json();
        if (result.status !== 200) {
          throw new Error(data.message);
        }
        this._currentUser = data;
      }
      this._token = token;
      this._waitingForAuth.forEach((_) => _(true));
    } catch (err: unknown) {
      console.error("loginWithToken() failed", { err });
      this._waitingForAuth.forEach((_) => _(false));
    } finally {
      this._waitingForAuth.clear();
    }

    if (refreshToken) {
      this._refreshToken = refreshToken;
      setInterval(async () => {
        if (this._refreshToken) {
          const result = await fetch(`${this._apiUrl}/v1/admin/auth/refresh`, {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({
              refreshToken: this._refreshToken,
            }),
          });
          const refreshData = await result.json();
          this._token = refreshData.authentication.accessToken;
        }
      }, 1000 * 60 * 60);
    }
  }

  isAuthenticated(): boolean {
    return this._token !== undefined;
  }

  async loginToPeer(peerUrl: string, password: string): Promise<void> {
    const result = await fetch(`${peerUrl}/login`, {
      method: "POST",
      body: JSON.stringify({ password }),
      headers: {
        "Content-Type": "application/json",
      },
    });

    if (result.status !== 200) {
      throw new LoginFailureError("Invalid authentication");
    }
  }

  /**
   * @deprecated use currentUser property instead.
   */
  getCurrentUser(): IUser | undefined {
    return this._currentUser;
  }

  async waitTilAuthenticated(): Promise<boolean> {
    if (this.token !== undefined) {
      return true;
    } else {
      return new Promise((resolve) => {
        this._waitingForAuth.add(resolve);
      });
    }
  }

  async listenForRefresh() {
    // refresh token every hour
    const hour = 1000 * 60 * 60;
    const askForFreshToken = () => {
      this._refreshTimer = undefined;
      this._refreshAuthToken();
    };

    this._addAccessTokenRefreshListener((token: string) => {
      if (this._refreshTimer) {
        // unless I get a fresh token sooner
        clearTimeout(this._refreshTimer);
      }
      this._refreshTimer = setTimeout(askForFreshToken, hour);
      this.loginWithToken(token);
    });

    // refresh token every hour
    this._refreshTimer = setTimeout(askForFreshToken, hour);
  }

  async forgotPassword(email: string) {
    await fetch(`${this._apiUrl}/v1/admin/auth/forgot-password`, {
      method: "POST",
      body: JSON.stringify({ email }),
      headers: {
        "Content-Type": "application/json",
      },
    });
  }

  /**
   * @example
   * // Body
   * await this.confirmForgotPassword({
   *     email: "joe@gmail.com"
   *     confirmationCode: "1",
   *     newPassword: "NewPassword"
   *   });
   */
  async confirmForgotPassword(request: IConfirmForgotPasswordRequest) {
    const response = await fetch(
      `${this._apiUrl}/v1/admin/auth/confirm-forgot-password`,
      {
        method: "POST",
        body: JSON.stringify(request),
        headers: {
          "Content-Type": "application/json",
        },
      }
    );

    return response.ok;
  }

  async respondToNewPasswordRequiredChallenge(
    request: IRespondToNewPasswordRequiredChallengeRequest
  ) {
    const response = await fetch(
      `${this._apiUrl}/v1/admin/auth/respond-to-new-password-required-challenge`,
      {
        method: "POST",
        body: JSON.stringify(request),
        headers: {
          "Content-Type": "application/json",
        },
      }
    );

    if (response.ok) {
      return await response.json();
    }

    throw new Error("respond-to-new-password-required-challenge failed");
  }

  async loginWithGoogle(token: string) {
    const response = await fetch(`${this._apiUrl}/v1/admin/auth/login-google`, {
      method: "POST",
      body: JSON.stringify(token),
      headers: {
        "Content-Type": "application/json",
      },
    });
    return await response.json();
  }

  async refresh(token: string) {
    const result = await fetch(`${this._apiUrl}/v1/admin/auth/refresh`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        refreshToken: token,
      }),
    });
    const refreshData = await result.json();
    await this.loginWithToken(refreshData.authentication.accessToken, token);
  }

  async checkSso(
    email: string,
    allowUserAutoCreation?: boolean
  ): Promise<ICheckSsoResult> {
    const result = await fetch(`${this._apiUrl}/v1/admin/auth/check-sso`, {
      method: "POST",
      body: JSON.stringify({ email, allowUserAutoCreation }),
      headers: {
        "Content-Type": "application/json",
      },
    });
    return await result.json();
  }

  async loginWithSso(ssoToken: string, ssoRefreshToken?: string) {
    const result = await fetch(`${this._apiUrl}/v1/admin/auth/login-sso`, {
      method: "POST",
      body: JSON.stringify({ token: ssoToken, refreshToken: ssoRefreshToken }),
      headers: {
        "Content-Type": "application/json",
      },
    });
    const ssoTokens = (await result.json()) as {
      authentication?: IAuthentication;
    };

    if (!ssoTokens.authentication) {
      throw new Error("Failed to login with SSO");
    }

    return await this.loginWithToken(
      ssoTokens.authentication.accessToken,
      ssoTokens.authentication.refreshToken
    );
  }
}
