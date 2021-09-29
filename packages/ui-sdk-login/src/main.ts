export class Login extends HTMLElement {
  static get observedAttributes() {
    return ["message", "error"];
  }
  constructor() {
    super();
  }
  connectedCallback() {
    this.innerHTML = `<div style="position: absolute; background: #1c1e2d; color: white; top: 0; left: 0; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center;">
    <form action="#" method="post" style="text-align: left">
    <div style="display: grid; grid-template-columns: 10rem 1fr; margin-bottom: 2rem; row-gap: 1rem;"><label for="email">Email: </label>
    <input id="email" name="email" type="email" placeholder=" " autocomplete="username" required>
    <label for="current-password">Password: </label><input id="current-password" name="current-password" type="password" autocomplete="current-password" aria-describedby="password-constraints" required/></div>
    <button large style="width: 100%" id="signin">Sign in</button><br><br>
    <div><span id="formant-login-message" style="color: #bac4e2">${
      this.getAttribute("message") ?? ""
    }</span><span id="formant-login-error" style="color: #ea719d">${
      this.getAttribute("error") ?? ""
    }</span></div></div></div>
    </form>
    `;
    const loginUsername = this.querySelector("#email") as HTMLInputElement;
    const loginPassword = this.querySelector(
      "#current-password"
    ) as HTMLInputElement;
    const loginButton = this.querySelector("form") as HTMLFormElement;
    loginButton.addEventListener("submit", (e) => {
      (e as Event).preventDefault();
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
    loginUsername.focus();
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
