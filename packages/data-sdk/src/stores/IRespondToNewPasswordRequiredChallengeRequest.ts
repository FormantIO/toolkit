export interface IRespondToNewPasswordRequiredChallengeRequest {
  userId: string;
  session: string;
  newPassword: string;
}
