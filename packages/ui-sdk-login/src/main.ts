import { Authentication, Fleet } from "@formant/data-sdk";

export class Login extends HTMLElement {
  constructor() {
    super();
  }
  connectedCallback() {
    this.innerHTML = `<div style="position: absolute; background: #1c1e2d; color: white; top: 0; left: 0; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center;">
    <div>Username: <input id="formant-login-username"/><br><br>Password: <input type="password" id="formant-login-password"/><br><br><button large style="width: 100%">Login</button><br><br>
    <div id="formant-login-error"></div></div></div>`;
    const loginUsername = this.querySelector(
      "#formant-login-username"
    ) as HTMLInputElement;
    const loginPassword = this.querySelector(
      "#formant-login-password"
    ) as HTMLInputElement;
    const loginButton = this.querySelector("button") as HTMLInputElement;
    const loginError = this.querySelector(
      "#formant-login-error"
    ) as HTMLInputElement;
    loginButton.addEventListener("click", async () => {
      loginError.style.color = "#bac4e2";
      loginError.innerHTML = "Connecting";
      try {
        await Authentication.login(loginUsername.value, loginPassword.value);
        this.remove();
      } catch (e: any) {
        loginError.style.color = "#ea719d";
        loginError.innerHTML = e.toString();
      }
    });
  }
}

customElements.define("formant-login", Login);
