import { IChallenge } from "../model/IChallenge";

export class LoginFailureError extends Error {
  readonly reason: string;

  constructor(reason: string) {
    super("login failed");
    this.reason = reason;
    this.name = "LoginFailureError";
    Object.setPrototypeOf(this, new.target.prototype); // restore prototype chain
  }
}

export class LoginChallengedError extends Error {
  readonly challenge: IChallenge;

  constructor(challenge: IChallenge) {
    super("login challenged");
    this.challenge = challenge;
    this.name = "LoginChallengedError";
    Object.setPrototypeOf(this, new.target.prototype); // restore prototype chain
  }
}
