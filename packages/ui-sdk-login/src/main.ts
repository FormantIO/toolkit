import { defined, definedAndNotNull } from "../../common/defined";

export class Login extends HTMLElement {
  constructor() {
    super();
  }
  connectedCallback() {
    this.innerHTML = `<div style="position: absolute; background: #1c1e2d; color: white; top: 0; left: 0; width: 100%; height: 100%; display: flex; align-items: center; justify-content: center;">
    <div>Username: <input/><br><br>Password: <input type="password"/><br><br><button large style="width: 100%">Login</button></div></div>`;
  }
}

customElements.define("formant-login", Login);
