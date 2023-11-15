// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  connectedCallback() {
    this.innerHTML = "<h1>I am a module!</h1>";

    window.setInterval(() => {
      const c = document.querySelector("formant-context");
      this.innerHTML = JSON.stringify(c.context);
    }, 1000);
  }
}

customElements.define("hello-module", HelloModule);
