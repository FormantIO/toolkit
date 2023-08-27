export interface IConfirmForgotPasswordRequest {
  email: string;
  confirmationCode: string;
  newPassword: string;
}
