export interface IChallenge {
  type: "new-password-required";
  userId: string;
  email: string;
  session: string;
}
