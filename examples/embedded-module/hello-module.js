// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  async getConfiguration() {
    const configurationAttribute = this.getAttribute("configuration");
    const contextEl = document.querySelector(`formant-context`);
    const client = contextEl.context.client;
    if (configurationAttribute !== null) {
      return await client.experimental.customModuleStore.getConfiguration(
        configurationAttribute
      );
    }
    return undefined;
  }

  connectedCallback() {
    this.innerHTML = "<h1>Retrieving configuration ...</h1>";

    this.getConfiguration().then((config) => {
      if (config) {
        this.innerHTML = `<h1>Configuration</h1><br>` + JSON.stringify(config);
      }
    });
  }
}

customElements.define("hello-module", HelloModule);
