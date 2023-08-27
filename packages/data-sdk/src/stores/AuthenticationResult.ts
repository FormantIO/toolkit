import { IChallenge } from "../model/IChallenge";
import { IAuthentication } from "./IAuthentication";

export interface AuthSuccessResult {
  result: "success";
  authentication: IAuthentication;
}

export interface AuthFailedResult {
  result: "failure";
  reason: string;
}

export interface AuthChallengeResult {
  result: "challenged";
  challenge: IChallenge;
}

export type AuthenticationResult =
  | AuthSuccessResult
  | AuthFailedResult
  | AuthChallengeResult;
