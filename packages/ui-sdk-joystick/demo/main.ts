import "../src/main";

const j = document.querySelector("formant-joystick");
const l = document.querySelector("#log");
if (j && l) {
  j.addEventListener("joystick", (e) => {
    const ce = e as CustomEvent;
    l.innerHTML = JSON.stringify(ce.detail) + "<br>" + l.innerHTML;
  });
}
