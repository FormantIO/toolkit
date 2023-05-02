import {
  User,
  IAuthentication,
  IAuthenticationStore,
  IConfirmForgotPasswordRequest,
  IRespondToNewPasswordRequiredChallengeRequest,
} from "./IAuthenticationStore";

interface IAuthenticationStoreOptions {
  apiUrl: string;

  refreshAuthToken: () => void;
  addAccessTokenRefreshListener: (callback: (token: string) => void) => void;
}

export class AuthenticationStore implements IAuthenticationStore {
  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  refreshToken: string | undefined;
  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  isShareToken: boolean = false;
  #currentOrganization: string | undefined;
  #currentUser: User | undefined;
  #defaultDeviceId: string | undefined;
  #token: string | undefined;
  readonly #apiUrl: string;
  readonly #refreshAuthToken: () => void;
  readonly #addAccessTokenRefreshListener: (
    callback: (token: string) => void
  ) => void;

  private waitingForAuth: ((result: boolean) => void)[] = [];

  constructor({
    apiUrl,
    refreshAuthToken,
    addAccessTokenRefreshListener,
  }: IAuthenticationStoreOptions) {
    this.#apiUrl = apiUrl;
    this.#refreshAuthToken = refreshAuthToken;
    this.#addAccessTokenRefreshListener = addAccessTokenRefreshListener;
  }

  get token(): string | undefined {
    return this.#token;
  }

  get currentUser(): User | undefined {
    return this.#currentUser;
  }

  get currentOrganization(): string | undefined {
    return this.#currentOrganization;
  }

  get defaultDeviceId(): string | undefined {
    return this.#defaultDeviceId;
  }

  async login(
    email: string,
    password: string
  ): Promise<IAuthentication | Error> {
    try {
      const result = await fetch(`${this.#apiUrl}/v1/admin/auth/login`, {
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
      await this.loginWithToken(
        auth.authentication.accessToken as string,
        auth.authentication.refreshToken as string
      );

      return auth.authentication;
    } catch (err: unknown) {
      console.error("login() failed", { err });
      this.waitingForAuth.forEach((_) => _(false));
      this.waitingForAuth = [];

      return Promise.reject(err);
    }
  }

  async loginWithToken(token: string, refreshToken?: string) {
    const tokenData = JSON.parse(atob(token.split(".")[1]));
    try {
      let userId: string | undefined;
      this.isShareToken =
        tokenData["formant:claims"] &&
        tokenData["formant:claims"].type == "share";
      if (tokenData["formant:claims"]) {
        this.#currentOrganization = tokenData["formant:claims"].organizationId;
      }
      if (tokenData["custom:organization_id"]) {
        this.#currentOrganization = tokenData["custom:organization_id"];
      }
      if (!this.isShareToken) {
        userId = tokenData.sub;
      }
      if (tokenData["formant:claims"] && tokenData["formant:claims"].userId) {
        userId = tokenData["formant:claims"].userId;
      }
      if (userId) {
        const result = await fetch(`${this.#apiUrl}/v1/admin/users/${userId}`, {
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
        this.#currentUser = data;
      }
      this.#token = token;
      this.waitingForAuth.forEach((_) => _(true));
    } catch (err: unknown) {
      console.error("loginWithToken() failed", { err });
      this.waitingForAuth.forEach((_) => _(false));
    }
    this.waitingForAuth = [];

    if (refreshToken) {
      this.refreshToken = refreshToken;
      setInterval(async () => {
        if (this.refreshToken) {
          const result = await fetch(`${this.#apiUrl}/v1/admin/auth/refresh`, {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({
              refreshToken: this.refreshToken,
            }),
          });
          const refreshData = await result.json();
          this.#token = refreshData.authentication.accessToken;
        }
      }, 1000 * 60 * 60);
    }
  }

  isAuthenticated(): boolean {
    return this.#token !== undefined;
  }

  /**
   * @deprecated use currentUser property instead.
   */
  getCurrentUser(): User | undefined {
    return this.#currentUser;
  }

  async waitTilAuthenticated(): Promise<boolean> {
    if (this.token !== undefined) {
      return true;
    } else {
      return new Promise((resolve) => {
        this.waitingForAuth.push(function (result: boolean) {
          resolve(result);
        });
      });
    }
  }

  async listenForRefresh() {
    // refresh token every hour
    this.#addAccessTokenRefreshListener((token: string) => {
      this.loginWithToken(token);
    });

    setInterval(async () => {
      this.#refreshAuthToken();
    }, 1000 * 60 * 60);
  }

  async forgotPassword(email: string) {
    await fetch(`${this.#apiUrl}/v1/admin/auth/forgot-password`, {
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
      `${this.#apiUrl}/v1/admin/auth/confirm-forgot-password`,
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
      `${
        this.#apiUrl
      }/v1/admin/auth/respond-to-new-password-required-challenge`,
      {
        method: "POST",
        body: JSON.stringify(request),
        headers: {
          "Content-Type": "application/json",
        },
      }
    );
    return await response.json();
  }

  async loginWithGoogle(token: string) {
    const response = await fetch(`${this.#apiUrl}/v1/admin/auth/login-google`, {
      method: "POST",
      body: JSON.stringify(token),
      headers: {
        "Content-Type": "application/json",
      },
    });
    return await response.json();
  }

  async refresh(token: string) {
    const result = await fetch(`${this.#apiUrl}/v1/admin/auth/refresh`, {
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
}
