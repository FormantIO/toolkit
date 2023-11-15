// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  connectedCallback() {
    this.innerHTML = "<h1>I am a module!</h1>";

    const configId = this.getAttribute("configurationid");
    if (configId) {
      const contextEl = document.querySelector(`formant-context`);
      (async () => {
        const config = await contextEl.context.client.getConfiguration(
          configId
        );
        this.innerHTML = `<h1>My configuration ${JSON.stringify(
          config,
          null,
          2
        )}!</h1>`;
      })();
    }
  }
}

customElements.define("hello-module", HelloModule);
