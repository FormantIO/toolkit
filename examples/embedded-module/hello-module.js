// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  connectedCallback() {
    this.innerHTML = "<h1>Retrieving configuration ...</h1>";

    const configurationAttribute = this.getAttribute("configuration");
    const contextEl = document.querySelector(`formant-context`);
    const client = contextEl.context.client;

    if (configurationAttribute !== null) {
      client.experimental.customModuleStore
        .getConfiguration(configurationAttribute)
        .then((config) => {
          this.innerHTML =
            `<h1>Configuration</h1><br>` + JSON.stringify(config);
        });
    }
  }
}

customElements.define("hello-module", HelloModule);
