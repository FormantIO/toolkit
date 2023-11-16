// create a module that just says hello

class HelloModule extends HTMLElement {
  constructor() {
    super();
  }

  connectedCallback() {
    this.innerHTML = "<h1>Retrieving health ...</h1>";

    const contextEl = document.querySelector(`formant-context`);
    const client = contextEl.context.client;
    setInterval(async () => {
      // last minute
      const start = new Date(Date.now() - 60 * 1000);
      const end = new Date();
      const results = await client.query(
        {},
        ["$.agent.health"],
        "health",
        start,
        end
      );
      if (results) {
        const health = await Promise.all(
          results.map((r) =>
            (async () => {
              const name = await client.getDeviceName(r.deviceId);
              return { name, latest: r.points[r.points.length - 1] };
            })()
          )
        );
        this.innerHTML = `<h1>health</h1>` + JSON.stringify(health, null, 2);
      }
    }, 1000);
  }
}

customElements.define("hello-module", HelloModule);
