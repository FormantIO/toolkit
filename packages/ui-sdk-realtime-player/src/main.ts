class RealtimePlayer extends HTMLElement {
  constructor() {
    super();
  }
  connectedCallback() {
    this.style.background = "#333";
  }
}

customElements.define("formant-realtime-player", RealtimePlayer);
