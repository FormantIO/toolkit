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

export interface IConfirmForgotPasswordRequest {
  email: string;
  confirmationCode: string;
  newPassword: string;
}

export interface IRespondToNewPasswordRequiredChallengeRequest {
  userId: string;
  session: string;
  newPassword: string;
}

export interface IAuthenticationStore {
  readonly token: string | undefined;
  readonly currentUser: User | undefined;
  readonly currentOrganization: string | undefined;
  readonly defaultDeviceId: string | undefined;

  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  readonly refreshToken: string | undefined;

  /**
   * @deprecated Do not use directly. This will be removed in future versions of the API
   */
  readonly isShareToken: boolean;

  login(email: string, password: string): Promise<IAuthentication | Error>;

  loginWithToken(token: string, refreshToken?: string): Promise<void>;

  isAuthenticated(): boolean;

  getCurrentUser(): User | undefined;

  waitTilAuthenticated(): Promise<boolean>;

  listenForRefresh(): Promise<void>;

  forgotPassword(email: string): Promise<void>;

  /**
   * @example
   * // Body
   * await this.confirmForgotPassword({
   *     email: "joe@gmail.com"
   *     confirmationCode: "1",
   *     newPassword: "NewPassword"
   *   });
   */
  confirmForgotPassword(
    request: IConfirmForgotPasswordRequest
  ): Promise<boolean>;

  respondToNewPasswordRequiredChallenge(
    request: IRespondToNewPasswordRequiredChallengeRequest
  ): Promise<any>;

  loginWithGoogle(token: string): Promise<any>;

  refresh(token: string): Promise<void>;
}
