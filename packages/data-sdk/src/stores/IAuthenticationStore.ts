/* eslint-disable @typescript-eslint/no-explicit-any */
import { IUser } from "../model/IUser";
import { AuthenticationResult } from "./AuthenticationResult";
import { IAuthentication } from "./IAuthentication";
import { IConfirmForgotPasswordRequest } from "./IConfirmForgotPasswordRequest";
import { IRespondToNewPasswordRequiredChallengeRequest } from "./IRespondToNewPasswordRequiredChallengeRequest";

export interface IAuthenticationStore {
  readonly token: string | undefined;
  readonly currentUser: IUser | undefined;
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

  set apiUrl(url: string);
  get apiUrl(): string;

  login(email: string, password: string): Promise<IAuthentication>;
  login(
    email: string,
    password: string,
    options: { advanced: true }
  ): Promise<AuthenticationResult>;

  loginWithToken(token: string, refreshToken?: string): Promise<void>;
  loginToPeer(
    peerUrl: string,
    username: string,
    password: string
  ): Promise<void>;

  isAuthenticated(): boolean;

  getCurrentUser(): IUser | undefined;

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
