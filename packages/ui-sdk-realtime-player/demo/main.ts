import "../src/main";

const j = document.querySelector("formant-joystick");
const l = document.querySelector("#log");
if (j && l) {
  j.addEventListener("joystick", (e) => {
    l.innerHTML = JSON.stringify(e.detail) + "<br>" + l.innerHTML;
  });
}
