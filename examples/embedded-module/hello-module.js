// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  connectedCallback() {
    this.innerHTML = "<h1>I am a module!</h1>";
  }
}

customElements.define("hello-module", HelloModule);
