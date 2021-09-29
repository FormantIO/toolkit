export class Login extends HTMLElement {
  static get observedAttributes() {
    return ["message", "error"];
  }
  constructor() {
    super();
  }
  connectedCallback() {
    this.innerHTML = `<div style="position: absolute; background: #1c1e2d; color: white; top: 0; left: 0; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center;">
    <div>Username: <input type="text" id="formant-login-username"/><br><br>Password: <input type="password" id="formant-login-password"/><br><br><button large style="width: 100%">Login</button><br><br>
    <div><span id="formant-login-message" style="color: #bac4e2">${
      this.getAttribute("message") ?? ""
    }</span><span id="formant-login-error" style="color: #ea719d">${
      this.getAttribute("error") ?? ""
    }</span></div></div></div>`;
    const loginUsername = this.querySelector(
      "#formant-login-username"
    ) as HTMLInputElement;
    const loginPassword = this.querySelector(
      "#formant-login-password"
    ) as HTMLInputElement;
    const loginButton = this.querySelector("button") as HTMLInputElement;
    loginButton.addEventListener("click", () => {
      this.dispatchEvent(
        new CustomEvent("login", {
          detail: {
            username: loginUsername.value,
            password: loginPassword.value,
          },
        })
      );
    });
    this.addEventListener("keyup", function (event) {
      if (event.keyCode === 13) {
        this.dispatchEvent(
          new CustomEvent("login", {
            detail: {
              username: loginUsername.value,
              password: loginPassword.value,
            },
          })
        );
      }
    });
  }
  attributeChangedCallback(name: string, _oldValue: string, newValue: string) {
    if (name === "message") {
      const loginMessage = this.querySelector(
        "#formant-login-message"
      ) as HTMLInputElement;
      loginMessage.innerHTML = newValue;
    }
    if (name === "error") {
      const loginError = this.querySelector(
        "#formant-login-error"
      ) as HTMLInputElement;
      loginError.innerHTML = newValue;
    }
  }
}

customElements.define("formant-login", Login);
