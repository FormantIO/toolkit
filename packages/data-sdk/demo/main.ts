import { Authentication } from "../src/main";
import "./style.css";

(async function () {
  const el = document.querySelector("#app");
  if (el) {
    try {
      await Authentication.confirmForgotPassword({
        email: "",
        confirmationCode: ``,
        newPassword: "",
      });
    } catch (error) {
      console.log(error);
    }
  }
})();
